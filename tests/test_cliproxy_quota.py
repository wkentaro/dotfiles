import io
import json
import os
import shutil
import socket
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request
from contextlib import redirect_stderr, redirect_stdout, suppress
from datetime import datetime, timezone
from http.client import BadStatusLine, IncompleteRead
from http.server import BaseHTTPRequestHandler, HTTPServer
from importlib.machinery import SourceFileLoader
from importlib.util import module_from_spec, spec_from_loader
from pathlib import Path
from unittest.mock import MagicMock, patch

SCRIPT = Path(__file__).resolve().parents[1] / "local/bin/cliproxy-quota"
LOADER = SourceFileLoader("cliproxy_quota", str(SCRIPT))
quota = module_from_spec(spec_from_loader(LOADER.name, LOADER))
with patch("sys.dont_write_bytecode", True):
    LOADER.exec_module(quota)
NOW = 1795484400
FIELDS = "/v0/management/auth-files/fields"
RESERVE = ["*", "cliproxy-quota-reserve"]


def account_report(path):
    status = quota.account_status(path.parent, path.name, 1)
    return quota.account_text(status), status["error"] is not None


def usage_lines(provider, data):
    return quota.window_lines(quota.quota_windows(provider, data))


def excluded_writes(requests):
    return [
        (body["name"], body["excluded_models"])
        for _, body in requests
        if "excluded_models" in body
    ]


def priority_writes(requests):
    return [
        (body["name"], body["priority"])
        for _, body in requests
        if "priority" in body
    ]


def claude_usage(weekly_left, time_left, five_hour=None):
    reset = datetime.fromtimestamp(NOW + time_left * 604800 / 100, timezone.utc)
    return {
        "five_hour": five_hour,
        "seven_day": {"utilization": 100 - weekly_left, "resets_at": reset.isoformat()},
    }


class QuotaTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.auth_dir = Path(self.directory.name) / "auth"
        self.auth_dir.mkdir()
        self.cache_dir = Path(self.directory.name) / "cache"
        self.own_accounts = Path(self.directory.name) / "own-accounts"
        for name, value in (
            ("CACHE_DIR", self.cache_dir),
            ("AUTH_DIR", self.auth_dir),
            ("OWN_ACCOUNTS", self.own_accounts),
        ):
            value_patch = patch.object(quota, name, value)
            value_patch.start()
            self.addCleanup(value_patch.stop)
        self.settings = Path(self.directory.name) / "settings-gateway.json"
        settings_patch = patch.object(quota, "GATEWAY_SETTINGS", self.settings)
        settings_patch.start()
        self.addCleanup(settings_patch.stop)
        env_patch = patch.dict(os.environ)
        env_patch.start()
        self.addCleanup(env_patch.stop)
        os.environ.pop("ANTHROPIC_BASE_URL", None)

    def account(self, provider="claude", **overrides):
        auth = {
            "type": provider,
            "email": "member@example.test",
            "access_token": "secret-access-token",
            "refresh_token": "secret-refresh-token",
        }
        auth.update(overrides)
        path = self.auth_dir / f"{provider}-{len(list(self.auth_dir.iterdir()))}.json"
        path.write_text(json.dumps(auth))
        return path

    def run_cli(self, *args):
        stdout, stderr = io.StringIO(), io.StringIO()
        with redirect_stdout(stdout), redirect_stderr(stderr):
            status = quota.main(list(args))
        return status, stdout.getvalue(), stderr.getvalue()

    def test_claude_remaining_and_missing_window(self):
        lines = usage_lines(
            "claude",
            {
                "five_hour": None,
                "seven_day": {"utilization": 62.5, "resets_at": "2026-11-19T07:00:00Z"},
                "seven_day_sonnet": {"utilization": 0, "resets_at": None},
                "seven_day_breakdown": {"opus": {}},
            },
        )
        self.assertEqual(len(lines), 2)
        self.assertIn("[########------------]  37.5% left", lines[0])
        self.assertIn("sonnet weekly", lines[1])
        self.assertIn("100.0% left", lines[1])
        self.assertIn("reset not reported", lines[1])

    def test_model_scoped_weekly_limits_are_shown_once(self):
        def scoped(name, percent, kind="weekly_scoped"):
            return {
                "kind": kind,
                "percent": percent,
                "resets_at": "2026-11-19T07:00:00Z",
                "scope": {"model": {"id": None, "display_name": name}, "surface": None},
            }

        windows = quota.quota_windows(
            "claude",
            {
                "seven_day": {"utilization": 24, "resets_at": "2026-11-19T07:00:00Z"},
                "seven_day_opus": {"utilization": 30, "resets_at": None},
                "limits": [
                    {"kind": "weekly_all", "percent": 24, "scope": None},
                    scoped("Fable", 21),
                    scoped("Opus", 99),
                    scoped("Haiku", 5, kind="session"),
                    {"kind": "weekly_scoped", "percent": 1, "scope": {"surface": "x"}},
                    "invalid",
                ],
            },
        )
        self.assertEqual(
            [label for label, _, _ in windows],
            ["weekly", "opus weekly", "Fable weekly"],
        )
        self.assertEqual(windows[-1][1:], (21, "2026-11-19T07:00:00Z"))

    def test_codex_labels_actual_window_duration(self):
        lines = usage_lines(
            "codex",
            {
                "rate_limit": {
                    "primary_window": {
                        "used_percent": 14,
                        "limit_window_seconds": 604800,
                        "reset_at": 1795484535,
                    },
                    "secondary_window": {
                        "used_percent": 100,
                        "limit_window_seconds": 18000,
                    },
                }
            },
        )
        self.assertIn("weekly", lines[0])
        self.assertIn("86.0% left", lines[0])
        self.assertIn("5-hour", lines[1])
        self.assertIn("[--------------------]   0.0% left", lines[1])

    def test_codex_weekly_only_and_missing_windows(self):
        lines = usage_lines(
            "codex",
            {
                "rate_limit": {
                    "primary_window": {
                        "used_percent": 0,
                        "limit_window_seconds": 604800,
                    },
                    "secondary_window": None,
                }
            },
        )
        self.assertEqual(len(lines), 1)
        self.assertIn("weekly", lines[0])
        with self.assertRaises(ValueError):
            quota.quota_windows("codex", {"rate_limit": {}})

    def test_invalid_percentages_are_not_shown_as_available_quota(self):
        for value in (None, True, "0", -1, 101, 10**1000, float("nan"), float("inf")):
            with self.subTest(value=value):
                self.assertEqual(quota.window_lines([("weekly", value, None)]), [])

    def test_duration_labels(self):
        for seconds, label in (
            (86400, "daily"),
            (10800, "3-hour"),
            (90, "90-second"),
            (120, "2-minute"),
            (None, "window"),
            (True, "window"),
            (float("inf"), "window"),
            (-1, "window"),
        ):
            self.assertEqual(quota.duration_label(seconds), label)
        for seconds in (604800, 86400, 18000, 10800, 120, 90):
            label = quota.duration_label(seconds)
            self.assertEqual(quota.window_seconds(label), seconds)

    def test_reset_time_converts_iso_and_epoch_to_local_timezone(self):
        expected = "resets " + datetime.fromtimestamp(
            1795484535, timezone.utc
        ).astimezone().strftime("%Y-%m-%d %H:%M %Z")
        self.assertEqual(quota.reset_text(1795484535), expected)
        self.assertEqual(quota.reset_text("2026-11-24T01:42:15Z"), expected)
        for value in (None, True, {}, "bad", "2026-11-19T07:00:00", 10**100):
            self.assertEqual(quota.reset_text(value), "reset not reported")

    def time_row(self, label, reset_in):
        now = 1795484535
        with patch.object(quota.time, "time", return_value=now):
            return quota.window_line(label, 40, now + reset_in)

    def test_time_row_shows_how_much_of_the_window_is_left(self):
        line = self.time_row("weekly", (4 * 24 + 2) * 3600)
        quota_row, time_row = line.split("\n")
        self.assertIn(
            "weekly           quota [############--------]  60.0% left", quota_row
        )
        self.assertIn(
            "time  [############--------]  58.3% left  4d 2h to reset (", time_row
        )
        self.assertEqual(quota_row.index("quota"), time_row.index("time"))
        quota_row, time_row = self.time_row("sonnet 4 5 weekly", 3600).split("\n")
        self.assertEqual(quota_row.index("quota"), time_row.index("time"))
        line = self.time_row("5-hour", (3 * 60 + 12) * 60)
        self.assertIn(
            "time  [#############-------]  64.0% left  3h 12m to reset (", line
        )
        self.assertIn("  0.0% left  0m to reset", self.time_row("sonnet weekly", -60))

    def test_no_time_row_without_a_window_length_or_a_valid_reset(self):
        for label, reset_in in (
            ("window", 3600),
            ("nan-hour", 3600),
            ("weekly", 10**9),
        ):
            with self.subTest(label=label):
                line = self.time_row(label, reset_in)
                self.assertNotIn("\n", line)
                self.assertIn("60.0% left  resets ", line)
        self.assertNotIn("\n", quota.window_line("weekly", 40, None))

    def test_text_and_json_show_the_5_hour_window_before_weekly(self):
        self.account()
        usage = {"five_hour": {"utilization": 10}, "seven_day": {"utilization": 40}}
        with patch.object(quota, "fetch_usage", return_value=usage):
            _, output, _ = self.run_cli()
            _, json_output, _ = self.run_cli("--json")
        self.assertIn("90.0% left", output)
        self.assertLess(output.index("5-hour"), output.index("weekly"))
        labels = [window["label"] for window in json.loads(json_output)[0]["windows"]]
        self.assertEqual(labels, ["5-hour", "weekly"])

    def test_json_lists_an_unreported_weekly_window_without_values(self):
        self.account()
        usage = {"five_hour": {"utilization": 10, "resets_at": "2026-11-19T07:00:00Z"}}
        with patch.object(quota, "fetch_usage", return_value=usage):
            _, output, _ = self.run_cli("--json")
        self.assertEqual(
            json.loads(output)[0]["windows"],
            [
                {"label": "5-hour", "used_percent": 10, "resets_at": "2026-11-19T07:00:00Z"},
                {"label": "weekly", "used_percent": None, "resets_at": None},
            ],
        )

    def test_fetch_uses_provider_endpoint_and_headers(self):
        auth = {"access_token": "secret-access-token", "account_id": "account-123"}
        for provider, expected_header, expected_value in (
            ("claude", "Anthropic-beta", "oauth-2025-04-20"),
            ("codex", "Chatgpt-account-id", "account-123"),
        ):
            with (
                self.subTest(provider=provider),
                patch.object(quota.urllib.request, "build_opener") as build,
            ):
                build.return_value.open.return_value.__enter__.return_value = (
                    io.StringIO("{}")
                )
                self.assertEqual(quota.fetch_usage(provider, auth, 4), {})
                request = build.return_value.open.call_args.args[0]
                self.assertEqual(request.full_url, quota.ENDPOINTS[provider])
                self.assertEqual(
                    request.get_header("Authorization"), "Bearer secret-access-token"
                )
                self.assertEqual(request.get_header(expected_header), expected_value)
                self.assertEqual(
                    build.return_value.open.call_args.kwargs, {"timeout": 4}
                )
                self.assertIsInstance(build.call_args.args[0], quota.NoRedirects)

    def test_redirects_are_not_followed_with_credentials(self):
        self.assertIsNone(
            quota.NoRedirects().redirect_request(
                None, None, 302, "Found", {}, "https://other.example.test/"
            )
        )

    def test_multiple_accounts_and_one_failure_leave_credentials_unchanged(self):
        claude = self.account()
        codex = self.account("codex", account_id="account-123")
        original = {p: p.read_bytes() for p in (claude, codex)}

        def response(provider, auth, timeout):
            if provider == "codex":
                raise urllib.error.HTTPError(
                    "https://example.test", 401, "secret-access-token", {}, io.BytesIO()
                )
            return {"seven_day": {"utilization": 25}}

        with patch.object(quota, "fetch_usage", side_effect=response):
            status, output, error = self.run_cli()
        self.assertEqual(status, 1)
        self.assertIn("75.0% left", output)
        self.assertIn("HTTP 401; re-login may be required", output)
        self.assertIn("claude  member@example.test", output)
        self.assertIn("codex  member@example.test", output)
        self.assertNotIn("secret-access-token", output + error)
        self.assertNotIn("secret-refresh-token", output + error)
        self.assertEqual({p: p.read_bytes() for p in original}, original)

    def test_disabled_and_unsupported_accounts_do_not_make_requests(self):
        self.account(disabled=True)
        self.account("other")
        with patch.object(quota, "fetch_usage") as fetch:
            status, output, error = self.run_cli()
        fetch.assert_not_called()
        self.assertEqual(status, 0)
        self.assertIn("Disabled", output)
        self.assertNotIn("other", output)

    def test_invalid_credentials_are_reported_without_tracebacks(self):
        (self.auth_dir / "broken.json").write_text(
            '{"access_token":"secret-access-token"'
        )
        (self.auth_dir / "array.json").write_text("[]")
        status, output, error = self.run_cli()
        self.assertEqual(status, 1)
        self.assertEqual(output.count("Cannot read credential JSON"), 2)
        self.assertNotIn("secret-access-token", output + error)

    def test_deeply_nested_credential_and_response_json_is_isolated(self):
        path = self.account()
        nested = "[" * 2000
        (self.auth_dir / "broken.json").write_text(nested)
        with patch.object(quota.urllib.request, "build_opener") as build:
            build.return_value.open.return_value.__enter__.return_value = io.StringIO(
                nested
            )
            output, failed = account_report(path)
        self.assertTrue(failed)
        self.assertIn("Invalid credentials or quota response", output)
        shutil.rmtree(self.cache_dir)
        with patch.object(quota, "fetch_usage", return_value={}) as fetch:
            status, output, error = self.run_cli()
        self.assertEqual(status, 1)
        self.assertIn("Cannot read credential JSON", output)
        self.assertIn("claude  member@example.test", output)
        self.assertEqual(fetch.call_count, 1)
        self.assertNotIn("secret-access-token", output + error)

    def test_network_and_response_errors_are_redacted(self):
        path = self.account()
        errors = [
            urllib.error.URLError("secret-access-token"),
            TimeoutError("secret-access-token"),
            IncompleteRead(b"secret-access-token"),
            BadStatusLine("secret-access-token"),
            ValueError("secret-access-token"),
            TypeError("secret-access-token"),
            RecursionError("secret-access-token"),
        ]
        for error in errors:
            shutil.rmtree(self.cache_dir, ignore_errors=True)
            with (
                self.subTest(error=error),
                patch.object(quota, "fetch_usage", side_effect=error) as fetch,
            ):
                output, failed = account_report(path)
                self.assertEqual(fetch.call_count, 1)
                self.assertTrue(failed)
                self.assertNotIn("secret-access-token", output)

    def test_truncated_response_does_not_hide_other_accounts(self):
        self.account()
        self.account("codex", account_id="account-123")

        def response(provider, auth, timeout):
            if provider == "claude":
                raise IncompleteRead(b"secret-access-token")
            return {
                "rate_limit": {
                    "primary_window": {
                        "used_percent": 10,
                        "limit_window_seconds": 604800,
                    }
                }
            }

        with patch.object(quota, "fetch_usage", side_effect=response):
            status, output, error = self.run_cli()
        self.assertEqual(status, 1)
        self.assertIn("Network request failed", output)
        self.assertIn("90.0% left", output)
        self.assertNotIn("secret-access-token", output + error)

    def test_missing_access_token_or_codex_account_id_makes_no_request(self):
        with patch.object(quota.urllib.request, "build_opener") as build:
            for provider, auth in (
                ("claude", {}),
                ("codex", {"access_token": "secret"}),
            ):
                with self.assertRaises(ValueError):
                    quota.fetch_usage(provider, auth, 1)
        build.assert_not_called()

    def test_invalid_provider_and_response_shapes(self):
        self.account(type={})
        status, output, error = self.run_cli()
        self.assertEqual(status, 1)
        self.assertIn("No Claude or Codex accounts", output)
        for provider, data in (
            ("claude", []),
            ("claude", {}),
            ("claude", {"five_hour": None, "seven_day": None}),
            ("codex", {}),
            ("codex", {"rate_limit": []}),
            ("codex", {"rate_limit": {"primary_window": None}}),
        ):
            with self.assertRaises(ValueError):
                quota.quota_windows(provider, data)

    def test_terminal_controls_are_removed_from_account_names(self):
        path = self.account(
            email="member\x1b[31m@example.test\nforged", organization_name="org\r\ttest"
        )
        with patch.object(quota, "fetch_usage", return_value={}):
            output, failed = account_report(path)
        self.assertNotIn("\x1b", output)
        self.assertNotIn("\r", output)
        self.assertNotIn("\t", output)
        self.assertIn("member?[31m@example.test?forged", output)

    def test_missing_directory_and_empty_directory(self):
        status, output, error = self.run_cli()
        self.assertEqual(status, 1)
        self.assertIn("No Claude or Codex accounts", output)
        status, output, error = self.run_cli(
            "--auth-dir", str(self.auth_dir / "missing")
        )
        self.assertEqual(status, 1)
        self.assertIn("Cannot read the credential directory", error)

    def test_timeout_requires_a_finite_positive_number(self):
        for value in ("0", "-1", "nan", "inf", "text"):
            with self.subTest(value=value), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    quota.main(["--timeout", value])
                self.assertEqual(error.exception.code, 2)

    def rate_limit(self, headers):
        return patch.object(
            quota,
            "fetch_usage",
            side_effect=urllib.error.HTTPError(
                "https://example.test", 429, "secret", headers, io.BytesIO()
            ),
        )

    def test_success_exit_and_rate_limit_error(self):
        self.account()
        with patch.object(
            quota, "fetch_usage", return_value={"seven_day": {"utilization": 0}}
        ):
            status, output, error = self.run_cli()
        self.assertEqual(status, 0)
        self.assertIn("100.0% left", output)
        for retry_after, expected in (
            (None, "5 min"),
            ("197", "4 min"),
            ("60", "1 min"),
            ("0", "1 min"),
            ("1", "1 min"),
            ("86400", "60 min"),
            ("Wed, 18 Nov 2026 07:00:00 GMT", "5 min"),
            ("9" * 5000, "5 min"),
        ):
            path = self.account()
            headers = {} if retry_after is None else {"Retry-After": retry_after}
            with self.rate_limit(headers):
                output, failed = account_report(path)
            self.assertTrue(failed)
            self.assertIn(f"HTTP 429; retry in {expected}", output)

    def report_at(self, path, now, response=None):
        response = response or {"seven_day": {"utilization": 0}}
        with (
            patch.object(quota.time, "time", return_value=now),
            patch.object(quota, "fetch_usage", return_value=response) as fetch,
        ):
            output, failed = account_report(path)
        return output, failed, fetch.call_count

    def cache_files(self, suffix):
        return list(self.cache_dir.glob("*" + suffix))

    def test_model_specific_claude_window_alone_is_reported(self):
        lines = usage_lines("claude", {"seven_day_opus": {"utilization": 20}})
        self.assertEqual(len(lines), 1)
        self.assertIn("opus weekly", lines[0])
        self.assertIn("80.0% left", lines[0])

    def test_fresh_results_are_reused_without_a_request(self):
        path = self.account()
        now = time.time()
        first, failed, calls = self.report_at(
            path, now, {"seven_day": {"utilization": 40}}
        )
        self.assertEqual(calls, 1)
        self.assertNotIn("as of", first)
        second, failed, calls = self.report_at(path, now + 299)
        self.assertEqual(calls, 0)
        self.assertFalse(failed)
        self.assertIn("as of", second)
        self.assertIn("60.0% left", second)
        third, failed, calls = self.report_at(path, now + 301)
        self.assertEqual(calls, 1)
        self.assertIn("100.0% left", third)

    def test_cache_is_private_and_holds_only_displayed_windows(self):
        self.cache_dir.mkdir(mode=0o755)
        self.cache_dir.chmod(0o755)
        path = self.account(email="member@example.test")
        self.report_at(
            path, time.time(), {"seven_day": {"utilization": 1}, "extra": "private"}
        )
        (cache,) = self.cache_files(".json")
        self.assertEqual(self.cache_dir.stat().st_mode & 0o777, 0o700)
        self.assertEqual(cache.stat().st_mode & 0o777, 0o600)
        self.assertNotIn("member", cache.name)
        for text in ("secret", "private", "member"):
            self.assertNotIn(text, cache.read_text())
        self.assertEqual(list(self.cache_dir.glob("tmp*")), [])

    def test_cache_is_not_shared_between_accounts_with_the_same_file_name(self):
        now = time.time()
        paths = []
        for name, provider, email in (
            ("one", "claude", "alice@example.test"),
            ("two", "claude", "bob@example.test"),
            ("three", "codex", "alice@example.test"),
        ):
            directory = self.auth_dir / name
            directory.mkdir()
            path = directory / "account.json"
            path.write_text(
                json.dumps({"type": provider, "email": email, "access_token": "secret"})
            )
            paths.append(path)
        self.report_at(paths[0], now, {"seven_day": {"utilization": 10}})
        output, failed, calls = self.report_at(
            paths[1], now + 1, {"seven_day": {"utilization": 70}}
        )
        self.assertEqual(calls, 1)
        self.assertIn("30.0% left", output)
        response = {
            "rate_limit": {
                "primary_window": {"used_percent": 0, "limit_window_seconds": 604800}
            }
        }
        output, failed, calls = self.report_at(paths[2], now + 2, response)
        self.assertEqual(calls, 1)
        self.assertFalse(failed)

    def test_failures_back_off_and_keep_showing_cached_quota(self):
        for failure, message, minutes, blocked, expired in (
            (self.rate_limit({"Retry-After": "120"}), "HTTP 429", 2, 400, 422),
            (
                patch.object(quota, "fetch_usage", side_effect=OSError("secret")),
                "Network request failed",
                1,
                340,
                362,
            ),
        ):
            with self.subTest(message=message):
                path = self.account()
                now = time.time()
                self.report_at(path, now, {"seven_day": {"utilization": 40}})
                with patch.object(quota.time, "time", return_value=now + 301), failure:
                    output, failed = account_report(path)
                self.assertTrue(failed)
                self.assertIn("as of", output)
                self.assertIn("60.0% left", output)
                self.assertIn(f"{message}; retry in {minutes} min", output)
                output, failed, calls = self.report_at(path, now + blocked)
                self.assertEqual(calls, 0)
                self.assertTrue(failed)
                self.assertIn("as of", output)
                self.assertIn("60.0% left", output)
                self.assertIn(f"{message}; retry in 1 min", output)
                output, failed, calls = self.report_at(path, now + expired)
                self.assertEqual(calls, 1)
                self.assertFalse(failed)
                self.assertIn("100.0% left", output)

    def test_backoff_counts_from_the_failure(self):
        path = self.account()
        now = time.time()
        clock = [now]

        def slow_failure(provider, auth, timeout):
            clock[0] += 70
            raise OSError("slow")

        with (
            patch.object(quota.time, "time", side_effect=lambda: clock[0]),
            patch.object(quota, "fetch_usage", side_effect=slow_failure),
        ):
            account_report(path)
        output, failed, calls = self.report_at(path, now + 75)
        self.assertEqual(calls, 0)
        self.assertIn("retry in 1 min", output)

    def test_changed_credentials_end_the_backoff_except_a_rate_limit(self):
        for failure, ended in (
            (patch.object(quota, "fetch_usage", side_effect=OSError("down")), True),
            (self.rate_limit({"Retry-After": "600"}), False),
        ):
            with self.subTest(ended=ended):
                path = self.account()
                now = time.time()
                with failure:
                    account_report(path)
                path.write_text(path.read_text().replace("secret", "renewed"))
                output, failed, calls = self.report_at(path, now + 1)
                self.assertEqual(calls, int(ended))
                self.assertEqual(failed, not ended)

    def test_pending_retry_is_cleaned_and_yields_to_fresh_quota(self):
        path = self.account()
        credential = quota.digest(path.read_text())
        now = time.time()
        self.report_at(path, now)
        (cached,) = self.cache_files(".json")
        cached.with_suffix(".retry").write_text(
            json.dumps([now + 400, "HTTP 500\x1b[31m", credential])
        )
        output, failed, calls = self.report_at(path, now + 1)
        self.assertEqual(calls, 0)
        self.assertFalse(failed)
        output, failed, calls = self.report_at(path, now + 301)
        self.assertEqual(calls, 0)
        self.assertTrue(failed)
        self.assertIn("HTTP 500?[31m; retry in 2 min", output)

    def test_unreadable_or_invalid_cache_is_ignored(self):
        now = time.time()
        credential = quota.digest(self.account().read_text())
        for suffix, cache in (
            (".json", "not json"),
            (".json", "[]"),
            (".json", '{"fetched_at": "now", "windows": [["weekly", 0, null]]}'),
            (".json", '{"fetched_at": 1e999, "windows": [["weekly", 0, null]]}'),
            (".json", json.dumps({"fetched_at": now + 60, "windows": []})),
            (".json", json.dumps({"fetched_at": now, "windows": []})),
            (".json", json.dumps({"fetched_at": now, "windows": [[1, 2, 3]]})),
            (".json", json.dumps({"fetched_at": now, "windows": [["weekly", 0]]})),
            (".json", json.dumps({"fetched_at": now, "windows": {"weekly": 0}})),
            (
                ".json",
                json.dumps({"fetched_at": now, "windows": [["weekly", None, None]]}),
            ),
            (".retry", json.dumps(now + 60)),
            (".retry", json.dumps([now + 60, "HTTP 500"])),
            (".retry", json.dumps([True, "HTTP 500", credential])),
            (".retry", json.dumps([now + 10**9, "HTTP 500", credential])),
            (".retry", json.dumps([10**400, "HTTP 500", credential])),
            (".retry", json.dumps([now + 60, None, credential])),
            (".retry", json.dumps([now + 60, "HTTP 500", "other"])),
            (".retry", json.dumps([now + 60, "HTTP 500", credential, 0])),
        ):
            with self.subTest(suffix=suffix, cache=cache):
                shutil.rmtree(self.cache_dir, ignore_errors=True)
                path = self.account()
                self.report_at(path, now - 1000)
                (cached,) = self.cache_files(".json")
                cached.with_suffix(suffix).write_text(cache)
                output, failed, calls = self.report_at(path, now)
                self.assertEqual(calls, 1)
                self.assertFalse(failed)
                self.assertIn("100.0% left", output)

    def test_unwritable_cache_does_not_fail_the_command(self):
        for target in ("mkstemp", "chmod"):
            with self.subTest(target=target):
                owner = quota.tempfile if target == "mkstemp" else type(self.cache_dir)
                with patch.object(owner, target, side_effect=OSError("denied")):
                    output, failed, calls = self.report_at(self.account(), time.time())
                self.assertEqual(calls, 1)
                self.assertFalse(failed)
                self.assertIn("100.0% left", output)

    def test_json_lists_every_account_with_parsed_windows(self):
        self.account(email="ok@example.test", organization_name="Org")
        self.account(email="down@example.test")
        self.account(email="off@example.test", disabled=True)
        self.account("codex", email="codex@example.test", account_id="id")
        (self.auth_dir / "broken.json").write_text("{")

        def response(provider, auth, timeout):
            if auth["email"] == "down@example.test":
                raise OSError("secret")
            if provider == "codex":
                window = {"used_percent": 25, "limit_window_seconds": 604800}
                return {"rate_limit": {"primary_window": window}}
            return {
                "five_hour": {"utilization": "bad", "resets_at": None},
                "seven_day": {
                    "utilization": 40,
                    "resets_at": "2026-11-19T07:00:00+00:00",
                },
            }

        with patch.object(quota, "fetch_usage", side_effect=response):
            status, output, error = self.run_cli("--json")
        self.assertEqual(status, 1)
        self.assertNotIn("secret", output)
        accounts = {account["file"]: account for account in json.loads(output)}
        self.assertEqual(len(accounts), 5)
        ok = accounts["claude-0.json"]
        self.assertEqual(ok["provider"], "claude")
        self.assertEqual(ok["email"], "ok@example.test")
        self.assertEqual(ok["organization"], "Org")
        self.assertIsNone(ok["error"])
        self.assertFalse(ok["cached"])
        self.assertTrue(ok["fetched_at"].endswith("Z"))
        self.assertEqual(
            ok["windows"],
            [
                {"label": "5-hour", "used_percent": None, "resets_at": None},
                {
                    "label": "weekly",
                    "used_percent": 40,
                    "resets_at": "2026-11-19T07:00:00Z",
                },
            ],
        )
        self.assertEqual(
            accounts["claude-1.json"]["error"], "Network request failed; retry in 1 min"
        )
        self.assertEqual(accounts["claude-1.json"]["windows"], [])
        self.assertTrue(accounts["claude-2.json"]["disabled"])
        codex = accounts["codex-3.json"]
        self.assertEqual(codex["windows"][0]["label"], "weekly")
        self.assertEqual(codex["windows"][0]["used_percent"], 25)
        self.assertIsNone(accounts["broken.json"]["provider"])
        self.assertEqual(
            accounts["broken.json"]["error"], "Cannot read credential JSON"
        )
        with patch.object(quota, "fetch_usage", side_effect=response) as fetch:
            status, output, error = self.run_cli("--json")
        cached = {account["file"]: account for account in json.loads(output)}
        self.assertTrue(cached["claude-0.json"]["cached"])
        self.assertEqual(cached["claude-0.json"]["fetched_at"], ok["fetched_at"])
        self.assertEqual(fetch.call_count, 0)

    def test_no_flags_hide_their_limits(self):
        self.account()
        usage = {
            "five_hour": {"utilization": 10, "resets_at": None},
            "seven_day": {"utilization": 40, "resets_at": None},
            "limits": [
                {
                    "kind": "weekly_scoped",
                    "percent": 21,
                    "scope": {"model": {"display_name": "Fable"}},
                }
            ],
        }
        for args, labels in (
            ((), ["5-hour", "weekly", "Fable weekly"]),
            (("--no-fable",), ["5-hour", "weekly"]),
            (("--no-5hour",), ["weekly", "Fable weekly"]),
            (("--no-5hour", "--no-fable"), ["weekly"]),
        ):
            with self.subTest(args=args):
                with patch.object(quota, "fetch_usage", return_value=usage):
                    _, output, _ = self.run_cli("--json", *args)
                windows = json.loads(output)[0]["windows"]
                self.assertEqual([window["label"] for window in windows], labels)
        with patch.object(quota, "fetch_usage", return_value=usage):
            _, output, _ = self.run_cli("--no-fable")
        self.assertNotIn("Fable", output)
        with patch.object(quota, "fetch_usage", return_value=usage):
            _, output, _ = self.run_cli("--no-5hour")
        self.assertNotIn("5-hour", output)
        self.assertIn("weekly", output)

    def test_json_times_are_utc_and_labels_are_cleaned(self):
        status = {
            "windows": [["x\x1b[31m weekly", 5, "2026-11-19T16:00:00+09:00"]],
            "fetched_at": 0,
        }
        result = quota.account_json(status)
        self.assertEqual(result["fetched_at"], "1970-01-01T00:00:00Z")
        self.assertEqual(
            result["windows"],
            [
                {
                    "label": "x?[31m weekly",
                    "used_percent": 5,
                    "resets_at": "2026-11-19T07:00:00Z",
                }
            ],
        )

    def test_out_of_range_reset_times_are_not_reported(self):
        self.addCleanup(time.tzset)
        with patch.dict(os.environ, {"TZ": "Asia/Tokyo"}):
            time.tzset()
            for value, utc in (
                ("9999-12-31T23:59:00-05:00", None),
                ("0001-01-01T00:00:00+09:00", None),
                ("9999-12-31T23:59:00Z", "9999-12-31T23:59:00Z"),
                (253402300799, "9999-12-31T23:59:59Z"),
            ):
                with self.subTest(value=value):
                    self.assertEqual(quota.reset_text(value), "reset not reported")
                    window = quota.account_json(
                        {"windows": [["weekly", 5, value]], "fetched_at": None}
                    )["windows"][0]
                    self.assertEqual(window["resets_at"], utc)

    def test_json_with_no_accounts_is_an_empty_list(self):
        status, output, error = self.run_cli("--json")
        self.assertEqual(status, 1)
        self.assertEqual(json.loads(output), [])

    def serve_in_background(self, server):
        self.addCleanup(server.server_close)
        thread = threading.Thread(target=server.serve_forever)
        thread.start()
        self.addCleanup(thread.join)
        self.addCleanup(server.shutdown)
        return server.server_address[1]

    def start_server(self, auth_dir):
        server = quota.quota_server(auth_dir, 1, 0)
        port = self.serve_in_background(server)
        self.assertEqual(server.server_address[0], "127.0.0.1")
        self.assertIs(type(server), quota.HTTPServer)
        return port

    def get_served(self, auth_dir, port=None):
        port = port or self.start_server(auth_dir)
        url = f"http://127.0.0.1:{port}/quota"
        try:
            with (
                redirect_stderr(io.StringIO()),
                urllib.request.urlopen(url, timeout=5) as response,
            ):
                return response.status, response.read()
        except urllib.error.HTTPError as error:
            return error.code, error.read()

    def test_serve_returns_the_json_accounts_without_identities(self):
        self.account(email="ok@example.test", organization_name="Org")
        self.account(email="off@example.test", disabled=True)
        usage = {"seven_day": {"utilization": 40, "resets_at": None}}
        with patch.object(quota, "fetch_usage", return_value=usage) as fetch:
            self.run_cli("--json")
            _, output, _ = self.run_cli("--json")
            code, body = self.get_served(self.auth_dir)
        self.assertEqual(code, 200)
        self.assertEqual(fetch.call_count, 1)
        for private in ("example.test", "Org", "claude-0", "secret"):
            self.assertNotIn(private.encode(), body)
        accounts = json.loads(body)
        for account in accounts:
            self.assertEqual(
                set(account),
                {
                    "provider",
                    "priority",
                    "disabled",
                    "windows",
                    "fetched_at",
                    "cached",
                    "error",
                },
            )
        expected = [
            {key: account[key] for key in accounts[0]} for account in json.loads(output)
        ]
        self.assertEqual(accounts, expected)
        self.assertEqual(expected[0]["windows"][-1]["used_percent"], 40)

    def test_serve_drops_an_idle_connection(self):
        with patch.object(quota, "SERVE_TIMEOUT", 0.2):
            port = self.start_server(self.auth_dir)
        idle = socket.create_connection(("127.0.0.1", port))
        self.addCleanup(idle.close)
        code, body = self.get_served(self.auth_dir, port)
        self.assertEqual((code, json.loads(body)), (200, []))

    def test_serve_command_serves_the_credential_directory_on_its_port(self):
        with patch.object(quota, "quota_server", return_value=MagicMock()) as server:
            self.run_cli("serve")
        server.assert_called_once_with(self.auth_dir, 10, 8318)
        server.return_value.serve_forever.assert_called_once_with()

    def test_serve_fails_without_a_readable_credential_directory(self):
        path = self.account()
        for auth_dir in (self.auth_dir / "missing", path):
            with self.subTest(auth_dir=auth_dir.name):
                code, _ = self.get_served(auth_dir)
                self.assertEqual(code, 503)
        if os.geteuid() != 0:
            self.addCleanup(self.auth_dir.chmod, 0o700)
            self.auth_dir.chmod(0)
            code, _ = self.get_served(self.auth_dir)
            self.assertEqual(code, 503)

    def test_no_local_accounts_reads_the_gateway(self):
        self.account(email="ok@example.test", organization_name="Org")
        self.account("codex", email="off@example.test", disabled=True)
        self.account(email="bad@example.test")
        usage = {"seven_day": {"utilization": 40, "resets_at": None}}
        empty = Path(self.directory.name) / "empty"
        empty.mkdir()

        def response(provider, auth, timeout):
            if auth["email"] == "bad@example.test":
                raise urllib.error.URLError("down")
            return usage

        fetch = patch.object(quota, "fetch_usage", side_effect=response)
        fetch.start()
        self.addCleanup(fetch.stop)
        port = self.start_server(self.auth_dir)
        self.get_served(self.auth_dir, port)
        url = f"http://127.0.0.1:{port}/"
        for source in ("environment", "settings"):
            with self.subTest(source=source):
                if source == "environment":
                    os.environ["ANTHROPIC_BASE_URL"] = url
                else:
                    os.environ.pop("ANTHROPIC_BASE_URL")
                    self.settings.write_text(
                        json.dumps({"env": {"ANTHROPIC_BASE_URL": url}})
                    )
                for auth_dir in (empty, empty / "missing"):
                    with patch.object(quota, "AUTH_DIR", auth_dir):
                        status, output, _ = self.run_cli()
                    self.assertEqual(status, 1)
                    self.assertNotIn("example.test", output)
                    self.assertNotIn("Org", output)
                    self.assertIn("claude  #1  as of", output)
                    self.assertIn("[############--------]  60.0% left", output)
                    self.assertIn("claude  #2\n  Network request failed", output)
                    self.assertIn("codex  #3\n  Disabled", output)
                    with patch.object(quota, "AUTH_DIR", auth_dir):
                        status, output, _ = self.run_cli("--json")
                    _, served = self.get_served(self.auth_dir, port)
                    self.assertEqual(json.loads(output), json.loads(served))

    def test_the_gateway_lists_accounts_in_place_in_line(self):
        for email, priority in (
            ("late@example.test", 1),
            ("unranked@example.test", None),
            ("soon@example.test", 30),
            ("mid@example.test", "5"),
        ):
            self.account(email=email, priority=priority)
        self.account("codex", email="codex@example.test")
        usage = {"seven_day": {"utilization": 40, "resets_at": None}}
        with patch.object(quota, "fetch_usage", return_value=usage):
            port = self.start_server(self.auth_dir)
            os.environ["ANTHROPIC_BASE_URL"] = f"http://127.0.0.1:{port}"
            with patch.object(quota, "AUTH_DIR", self.auth_dir / "missing"):
                _, output, _ = self.run_cli()
        self.assertEqual(
            [line for line in output.splitlines() if line.startswith(("claude", "codex"))],
            [
                "claude  #1  priority 1",
                "claude  #2  priority 2",
                "claude  #3  priority 3",
                "claude  #4",
                "codex  #5",
            ],
        )

    def test_local_accounts_win_over_the_gateway(self):
        path = self.account()
        os.environ["ANTHROPIC_BASE_URL"] = "http://127.0.0.1:9"
        usage = {"seven_day": {"utilization": 40, "resets_at": None}}
        with patch.object(quota, "fetch_usage", return_value=usage):
            status, output, _ = self.run_cli()
        self.assertEqual(status, 0)
        self.assertIn("member@example.test", output)
        missing = self.auth_dir / "missing"
        for auth_dir in (missing, path):
            status, _, error = self.run_cli("--auth-dir", str(auth_dir))
            self.assertEqual(status, 1)
            self.assertIn("Cannot read the credential directory", error)
        with patch.object(quota, "AUTH_DIR", missing):
            status, _, error = self.run_cli("--auth-dir", str(missing))
            self.assertEqual(status, 1)
            self.assertIn("Cannot read the credential directory", error)
        empty = Path(self.directory.name) / "empty"
        empty.mkdir()
        status, output, _ = self.run_cli("--auth-dir", str(empty))
        self.assertEqual(status, 1)
        self.assertIn("No Claude or Codex accounts", output)
        if os.geteuid() != 0:
            self.addCleanup(self.auth_dir.chmod, 0o700)
            self.auth_dir.chmod(0)
            status, _, error = self.run_cli()
            self.assertEqual(status, 1)
            self.assertIn("Cannot read the credential directory", error)

    def test_unusable_gateway_settings_are_ignored(self):
        for settings in (
            "[1]",
            '"url"',
            '{"env": ["x"]}',
            '{"env": {"ANTHROPIC_BASE_URL": 5}}',
            "{",
        ):
            with self.subTest(settings=settings):
                self.settings.write_text(settings)
                status, output, _ = self.run_cli()
                self.assertEqual(status, 1)
                self.assertIn("No Claude or Codex accounts", output)
        self.settings.write_text(
            '{"env": {"ANTHROPIC_BASE_URL": "http://gateway.test"}}'
        )
        os.environ["ANTHROPIC_BASE_URL"] = ""
        with patch.object(quota.urllib.request, "urlopen") as urlopen:
            status, output, _ = self.run_cli()
        urlopen.assert_not_called()
        self.assertEqual(status, 1)
        self.assertIn("No Claude or Codex accounts", output)

    def test_unreachable_or_invalid_gateway_fails(self):
        os.environ["ANTHROPIC_BASE_URL"] = "http://gateway\x1b.test/"
        bodies = [b"{}", b"[1]", b"not json", b"[" * 100000, b"\xff"]
        for body in bodies:
            with self.subTest(body=body[:10]):
                with patch.object(
                    quota.urllib.request, "urlopen", return_value=io.BytesIO(body)
                ):
                    status, output, error = self.run_cli()
                self.assertEqual((status, output), (1, ""))
                self.assertEqual(error, "Cannot read http://gateway?.test/quota.\n")
        for failure in (
            urllib.error.HTTPError("http://gateway.test", 502, "Bad", {}, None),
            IncompleteRead(b"["),
        ):
            with patch.object(quota.urllib.request, "urlopen", side_effect=failure):
                status, _, error = self.run_cli()
            self.assertEqual(status, 1)
            self.assertIn("Cannot read", error)

    def test_hostile_gateway_accounts_are_cleaned(self):
        os.environ["ANTHROPIC_BASE_URL"] = "http://gateway.test"
        accounts = [
            {
                "provider": "claude\x1b[31m",
                "windows": [{"label": "weekly\x1b", "used_percent": "50"}, 1],
                "fetched_at": "9999-12-31T23:59:59Z",
                "cached": True,
                "error": 5,
                "disabled": "yes",
                "priority": "1\x1b",
            },
            {
                "provider": 5,
                "windows": 5,
                "fetched_at": 10**400,
                "cached": "yes",
                "error": "bad\x07",
                "priority": True,
            },
            {"provider": "codex", "fetched_at": -1, "priority": -3},
        ]
        body = json.dumps(accounts).encode()
        with patch.object(
            quota.urllib.request, "urlopen", return_value=io.BytesIO(body)
        ):
            status, output, _ = self.run_cli()
        self.assertEqual(status, 1)
        self.assertEqual(
            output,
            "claude?[31m  #1\n\n#2\n  bad?\n\n"
            "codex  #3\n",
        )
        with patch.object(
            quota.urllib.request, "urlopen", return_value=io.BytesIO(body)
        ):
            _, output, _ = self.run_cli("--json")
        self.assertEqual(
            json.loads(output),
            [
                {
                    "provider": "claude?[31m",
                    "priority": None,
                    "disabled": False,
                    "windows": [
                        {"label": "weekly?", "used_percent": None, "resets_at": None}
                    ],
                    "fetched_at": None,
                    "cached": True,
                    "error": None,
                },
                {
                    "provider": None,
                    "priority": None,
                    "disabled": False,
                    "windows": [],
                    "fetched_at": None,
                    "cached": False,
                    "error": "bad?",
                },
                {
                    "provider": "codex",
                    "priority": None,
                    "disabled": False,
                    "windows": [],
                    "fetched_at": None,
                    "cached": False,
                    "error": None,
                },
            ],
        )


    def start_management(self, files, gone=()):
        key_path = Path(self.directory.name) / "management-key"
        key_path.write_text("management-secret\n")
        requests = []
        auth_dir = self.auth_dir
        memory = {}
        for file in files:
            with suppress(OSError, ValueError):
                memory[file["name"]] = json.loads((auth_dir / file["name"]).read_text())

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def reply(self, code, value):
                body = json.dumps(value).encode()
                self.send_response(code)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.headers["Authorization"] != "Bearer management-secret":
                    return self.reply(401, {"error": "invalid management key"})
                self.reply(200, {"files": files})

            def do_PATCH(self):
                length = int(self.headers["Content-Length"])
                body = json.loads(self.rfile.read(length))
                requests.append((self.path, body))
                if body["name"] in gone:
                    return self.reply(404, {"error": "auth file not found"})
                models = body.get("excluded_models")
                if models is not None and not (
                    isinstance(models, list)
                    and all(isinstance(model, str) for model in models)
                ):
                    message = "excluded_models must be a string array or null"
                    return self.reply(400, {"error": message})
                for file in files:
                    if file["name"] == body["name"]:
                        file.update(body)
                if body["name"] in memory:
                    auth = memory[body["name"]]
                    if "excluded_models" in body:
                        auth.pop("excluded-models", None)
                    auth.update({k: v for k, v in body.items() if k != "name"})
                    (auth_dir / body["name"]).write_text(json.dumps(auth))
                self.reply(200, {"status": "ok"})

        port = self.serve_in_background(HTTPServer(("127.0.0.1", 0), Handler))
        url = f"http://127.0.0.1:{port}/v0/management/auth-files"
        for name, value in (("MANAGEMENT_URL", url), ("MANAGEMENT_KEY", key_path)):
            value_patch = patch.object(quota, name, value)
            value_patch.start()
            self.addCleanup(value_patch.stop)
        return requests

    def weekly_usage(self, hours_by_email, used_by_email=None):
        def response(provider, auth, timeout):
            hours = hours_by_email[auth["email"]]
            if hours is None:
                raise urllib.error.URLError("offline")
            reset = datetime.fromtimestamp(time.time() + hours * 3600, timezone.utc)
            weekly = {"resets_at": reset.isoformat()}
            used = (used_by_email or {}).get(auth["email"], 10)
            if used is not None:
                weekly["utilization"] = used
            return {"seven_day": weekly}

        return patch.object(quota, "fetch_usage", side_effect=response)

    def prioritize_at(self, now, usage_by_email):
        def response(provider, auth, timeout):
            usage = usage_by_email[auth["email"]]
            if isinstance(usage, Exception):
                raise usage
            return usage

        with (
            patch.object(quota.time, "time", return_value=now),
            patch.object(quota, "fetch_usage", side_effect=response),
        ):
            return self.run_cli("prioritize")

    def test_prioritize_puts_the_soonest_weekly_reset_first(self):
        hours = {"late@x.test": 100, "soon@x.test": 2, "mid@x.test": 30, "down@x.test": None}
        names = [self.account(email=email).name for email in hours]
        codex = self.account("codex", email="codex@x.test").name
        files = [{"name": name, "priority": None} for name in names]
        files[2]["priority"] = 2
        files[3]["priority"] = 4
        files.append({"name": codex, "priority": None})
        requests = self.start_management(files)
        with self.weekly_usage(hours):
            status, output, _ = self.run_cli("prioritize")
        self.assertEqual(status, 0)
        self.assertEqual(
            requests,
            [
                ("/v0/management/auth-files/fields", {"name": names[0], "priority": 1}),
                ("/v0/management/auth-files/fields", {"name": names[1], "priority": 3}),
                ("/v0/management/auth-files/fields", {"name": names[3], "priority": 0}),
            ],
        )
        self.assertIn(f"{names[1]}  priority 0 -> 3", output)

    def test_prioritize_ranks_used_up_accounts_last(self):
        hours = {"spent@x.test": 2, "late@x.test": 100, "low@x.test": 30}
        names = [self.account(email=email).name for email in hours]
        requests = self.start_management(
            [{"name": name, "priority": priority} for name, priority in zip(names, (3, 1, 2))]
        )
        with self.weekly_usage(hours, {"spent@x.test": 100, "low@x.test": 99}):
            status, output, _ = self.run_cli("prioritize")
        self.assertEqual(
            (status, output, requests),
            (
                0,
                f"{names[0]}  priority 3 -> 0\n",
                [("/v0/management/auth-files/fields", {"name": names[0], "priority": 0})],
            ),
        )

    def test_prioritize_ranks_accounts_with_unknown_weekly_usage_last(self):
        hours = {"unknown@x.test": 3, "half@x.test": 12}
        names = [self.account(email=email).name for email in hours]
        requests = self.start_management([{"name": name, "priority": 0} for name in names])
        with self.weekly_usage(hours, {"unknown@x.test": None, "half@x.test": 50}):
            status, output, _ = self.run_cli("prioritize")
        self.assertEqual(
            (status, output, requests),
            (
                0,
                f"{names[1]}  priority 0 -> 1\n",
                [("/v0/management/auth-files/fields", {"name": names[1], "priority": 1})],
            ),
        )

    def test_prioritize_leaves_unchanged_priorities_alone(self):
        name = self.account(email="only@x.test").name
        requests = self.start_management([{"name": name, "priority": 1}])
        with self.weekly_usage({"only@x.test": 5}):
            status, output, _ = self.run_cli("prioritize")
        self.assertEqual((status, output, requests), (0, "", []))

    def test_weekly_resets_within_a_minute_rank_by_file_name(self):
        now = 1794960000
        statuses = [
            {"provider": "claude", "file": file, "windows": [("weekly", 0, now + offset)]}
            for file, offset in (("b.json", 3600), ("a.json", 3610), ("c.json", -60))
        ]
        self.assertEqual(
            quota.drain_priorities(statuses, now),
            {"a.json": 2, "b.json": 1, "c.json": 0},
        )

    def test_prioritize_closes_a_teammate_account_below_its_weekly_reserve(self):
        self.own_accounts.write_text("me@x.test\n")
        me = self.account(email="me@x.test").name
        mate = self.account(email="mate@x.test").name
        requests = self.start_management(
            [{"name": me, "priority": 0}, {"name": mate, "priority": 1}]
        )
        usage = {"me@x.test": claude_usage(90, 70), "mate@x.test": claude_usage(40, 60)}
        status, output, _ = self.prioritize_at(NOW, usage)
        self.assertEqual(
            (status, output),
            (
                0,
                f"{mate}  priority 1 -> 0\n"
                f'{mate}  excluded_models [] -> ["*", "cliproxy-quota-reserve"]\n'
                f"{me}  priority 0 -> 1\n",
            ),
        )
        self.assertEqual(
            requests,
            [
                (FIELDS, {"name": mate, "priority": 0, "excluded_models": RESERVE}),
                (FIELDS, {"name": me, "priority": 1}),
            ],
        )
        with patch.object(quota.time, "time", return_value=NOW):
            _, output, _ = self.run_cli()
        me_text, mate_text = output.split("\n\n")
        self.assertNotIn("Reserved", me_text)
        self.assertTrue(mate_text.startswith("claude  mate@x.test"))
        self.assertTrue(mate_text.endswith("\n  Reserved for its owner\n"))

    def test_only_a_claude_account_closed_by_prioritize_shows_reserved(self):
        self.account(email="closed@x.test", excluded_models=RESERVE)
        self.account(email="hand-set@x.test", excluded_models=["*"])
        for email, models in (("codex@x.test", ["*"]), ("marked@x.test", RESERVE)):
            self.account("codex", email=email, account_id="id", excluded_models=models)
        usage = {
            "seven_day": {"utilization": 40},
            "rate_limit": {"primary_window": {"used_percent": 1}},
        }
        with patch.object(quota, "fetch_usage", return_value=usage):
            _, output, _ = self.run_cli()
            _, json_output, _ = self.run_cli("--json")
        self.assertEqual(
            [
                (account["email"], account["reserved"], "excluded_models" in account)
                for account in json.loads(json_output)
            ],
            [
                ("closed@x.test", True, False),
                ("hand-set@x.test", False, False),
                ("codex@x.test", False, False),
                ("marked@x.test", False, False),
            ],
        )
        self.assertEqual(
            [
                account.splitlines()[0]
                for account in output.split("\n\n")
                if "Reserved for its owner" in account
            ],
            ["claude  closed@x.test"],
        )

    def test_prioritize_reopens_a_teammate_account_that_recovered(self):
        self.own_accounts.write_text("me@x.test\n")
        mate = self.account(email="mate@x.test", excluded_models=RESERVE).name
        requests = self.start_management([{"name": mate, "priority": 0}])
        status, output, _ = self.prioritize_at(
            NOW, {"mate@x.test": claude_usage(80, 50)}
        )
        self.assertEqual(
            (status, output, requests),
            (
                0,
                f"{mate}  priority 0 -> 1\n"
                f'{mate}  excluded_models ["*", "cliproxy-quota-reserve"] -> null\n',
                [(FIELDS, {"name": mate, "priority": 1, "excluded_models": None})],
            ),
        )

    def test_a_teammate_account_needs_half_its_5_hour_quota_left(self):
        self.own_accounts.write_text("me@x.test\n")
        low = self.account(email="low@x.test").name
        half = self.account(email="half@x.test").name
        requests = self.start_management(
            [{"name": low, "priority": 0}, {"name": half, "priority": 1}]
        )
        usage = {
            "low@x.test": claude_usage(90, 50, {"utilization": 51, "resets_at": None}),
            "half@x.test": claude_usage(90, 60, {"utilization": 50, "resets_at": None}),
        }
        status, _, _ = self.prioritize_at(NOW, usage)
        self.assertEqual(
            (status, requests),
            (0, [(FIELDS, {"name": low, "excluded_models": RESERVE})]),
        )

    def test_an_idle_5_hour_window_counts_as_unused(self):
        self.own_accounts.write_text("me@x.test\n")
        idle = {
            "null@x.test": None,
            "zero@x.test": {"utilization": 0.0, "resets_at": None},
            "unset@x.test": {"utilization": None, "resets_at": None},
        }
        names = [
            self.account(email=email, excluded_models=RESERVE).name for email in idle
        ]
        requests = self.start_management([{"name": name, "priority": 0} for name in names])
        usage = {
            email: claude_usage(90, 50, five_hour) for email, five_hour in idle.items()
        }
        status, _, _ = self.prioritize_at(NOW, usage)
        self.assertEqual(
            (status, excluded_writes(requests)),
            (0, [(name, None) for name in names]),
        )

    def test_unknown_5_hour_usage_closes_a_teammate_account(self):
        self.own_accounts.write_text("me@x.test\n")
        absent = self.account(email="absent@x.test").name
        unknown = self.account(email="unknown@x.test").name
        requests = self.start_management(
            [{"name": absent, "priority": 0}, {"name": unknown, "priority": 0}]
        )
        no_five_hour = claude_usage(90, 50)
        del no_five_hour["five_hour"]
        in_an_hour = datetime.fromtimestamp(NOW + 3600, timezone.utc).isoformat()
        usage = {
            "absent@x.test": no_five_hour,
            "unknown@x.test": claude_usage(
                90, 50, {"utilization": None, "resets_at": in_an_hour}
            ),
        }
        status, _, _ = self.prioritize_at(NOW, usage)
        self.assertEqual(
            (status, excluded_writes(requests)),
            (0, [(absent, RESERVE), (unknown, RESERVE)]),
        )
        with patch.object(quota.time, "time", return_value=NOW):
            _, output, _ = self.run_cli()
        self.assertNotIn("5-hour", output)

    def test_weekly_quota_left_equal_to_the_time_left_closes_a_teammate_account(self):
        self.own_accounts.write_text("me@x.test\n")
        mate = self.account(email="mate@x.test").name
        requests = self.start_management([{"name": mate, "priority": 1}])
        status, _, _ = self.prioritize_at(NOW, {"mate@x.test": claude_usage(29, 29)})
        self.assertEqual(
            (status, requests),
            (
                0,
                [(FIELDS, {"name": mate, "priority": 0, "excluded_models": RESERVE})],
            ),
        )

    def test_a_stale_reading_after_a_failed_refresh_keeps_a_teammate_account_closed(self):
        self.own_accounts.write_text("me@x.test\n")
        emails = ("open@x.test", "closed@x.test")
        for code, later in ((503, 20 * 60), (429, 604800 / 4)):
            with self.subTest(code=code):
                for path in self.auth_dir.iterdir():
                    path.unlink()
                shutil.rmtree(self.cache_dir, ignore_errors=True)
                opened = self.account(email=emails[0]).name
                closed = self.account(email=emails[1], excluded_models=RESERVE).name
                with (
                    patch.object(quota.time, "time", return_value=NOW),
                    patch.object(quota, "fetch_usage", return_value=claude_usage(90, 90)),
                ):
                    self.run_cli()
                requests = self.start_management(
                    [{"name": opened, "priority": 0}, {"name": closed, "priority": 0}]
                )
                errors = {
                    email: urllib.error.HTTPError(
                        "https://example.test", code, "failed", {}, io.BytesIO()
                    )
                    for email in emails
                }
                status, _, _ = self.prioritize_at(NOW + later, errors)
                self.assertEqual(
                    (status, requests),
                    (0, [(FIELDS, {"name": opened, "excluded_models": RESERVE})]),
                )

    def test_a_reading_past_its_weekly_reset_keeps_a_teammate_account_closed(self):
        self.own_accounts.write_text("me@x.test\n")
        opened = self.account(email="open@x.test").name
        closed = self.account(email="closed@x.test", excluded_models=RESERVE).name
        requests = self.start_management(
            [{"name": opened, "priority": 0}, {"name": closed, "priority": 0}]
        )
        past = claude_usage(90, -1)
        status, _, _ = self.prioritize_at(
            NOW, {"open@x.test": past, "closed@x.test": past}
        )
        self.assertEqual(
            (status, requests),
            (0, [(FIELDS, {"name": opened, "excluded_models": RESERVE})]),
        )

    def test_prioritize_never_closes_own_accounts(self):
        self.own_accounts.write_text("\n  Me@X.test \n\n")
        me = self.account(email="me@x.test").name
        mate = self.account(email="mate@x.test").name
        (self.auth_dir / "broken.json").write_text("{")
        requests = self.start_management(
            [
                {"name": me, "priority": 0},
                {"name": mate, "priority": 0},
                {"name": "broken.json", "priority": 3},
            ]
        )
        spent = claude_usage(0, 50, {"utilization": 100, "resets_at": None})
        status, _, _ = self.prioritize_at(
            NOW, {"me@x.test": spent, "mate@x.test": spent}
        )
        self.assertEqual(
            (status, requests),
            (0, [(FIELDS, {"name": mate, "excluded_models": RESERVE})]),
        )

    def test_without_own_accounts_every_account_is_yours(self):
        spent = self.account(email="spent@x.test").name
        closed = self.account(email="closed@x.test", excluded_models=RESERVE).name
        requests = self.start_management(
            [{"name": spent, "priority": 0}, {"name": closed, "priority": 0}]
        )
        usage = {"spent@x.test": claude_usage(0, 50), "closed@x.test": claude_usage(0, 50)}
        status, _, _ = self.prioritize_at(NOW, usage)
        self.assertEqual(
            (status, requests),
            (0, [(FIELDS, {"name": closed, "excluded_models": None})]),
        )

    def test_prioritize_never_reopens_a_hand_set_exclusion(self):
        hand_set = {
            "every@x.test": ["*"],
            "text@x.test": "*",
            "opus@x.test": ["claude-opus-*"],
            "spent@x.test": ["*"],
        }
        names = [
            self.account(email=email, excluded_models=models).name
            for email, models in hand_set.items()
        ]
        requests = self.start_management([{"name": name, "priority": 0} for name in names])
        usage = {email: claude_usage(90, 50) for email in hand_set}
        usage["spent@x.test"] = claude_usage(0, 50)
        without_own_accounts, _, _ = self.prioritize_at(NOW, usage)
        self.own_accounts.write_text("me@x.test\n")
        with_own_accounts, _, _ = self.prioritize_at(NOW, usage)
        self.assertEqual(
            (without_own_accounts, with_own_accounts, excluded_writes(requests)),
            (0, 0, []),
        )

    def test_a_hand_set_wildcard_counts_after_trimming_as_cliproxyapi_does(self):
        self.own_accounts.write_text("me@x.test\n")
        hand_set = {
            "spaced@x.test": [" * "],
            "wide@x.test": ["\t*　"],
            "text@x.test": "claude-opus-*, *",
            "stars@x.test": ["**"],
            "separator@x.test": ["\x1c*"],
        }
        names = {
            email: self.account(email=email, excluded_models=models).name
            for email, models in hand_set.items()
        }
        requests = self.start_management(
            [{"name": name, "priority": 0} for name in names.values()]
        )
        status, _, _ = self.prioritize_at(
            NOW, {email: claude_usage(0, 50) for email in hand_set}
        )
        self.assertEqual(
            (status, excluded_writes(requests)),
            (0, [(names["separator@x.test"], ["\x1c*", *RESERVE])]),
        )

    def test_a_mixed_case_type_is_reserved_as_cliproxyapi_reads_it(self):
        self.own_accounts.write_text("me@x.test\n")
        path = self.account(provider=" Claude ", email="type@x.test")
        name = path.rename(self.auth_dir / "claude-type.json").name
        requests = self.start_management([{"name": name, "priority": 0}])
        status, _, _ = self.prioritize_at(NOW, {"type@x.test": claude_usage(10, 50)})
        self.assertEqual((status, excluded_writes(requests)), (0, [(name, RESERVE)]))

    def test_teammate_files_cliproxyapi_does_not_watch_are_reported_without_writes(self):
        self.own_accounts.write_text("me@x.test\n")
        listed = {"suffix.JSON": "suffix@x.test", "link.json": "link@x.test"}
        for name, email in (("suffix.JSON", "suffix@x.test"), ("target", "link@x.test")):
            (self.auth_dir / name).write_text(json.dumps({"type": "claude", "email": email}))
        (self.auth_dir / "link.json").symlink_to(self.auth_dir / "target")
        requests = self.start_management(
            [
                {"name": name, "provider": "claude", "email": email, "priority": 0}
                for name, email in listed.items()
            ]
        )
        with patch.object(quota, "fetch_usage", return_value=claude_usage(10, 50)) as fetch:
            status, _, error = self.run_cli("prioritize")
        self.assertEqual((status, requests, fetch.call_count), (1, [], 0))
        self.assertIn('Cannot reserve "suffix.JSON"', error)
        self.assertIn('Cannot reserve "link.json"', error)

    def test_an_unreadable_teammate_credential_fails_the_run(self):
        self.own_accounts.write_text("me@x.test\n")
        (self.auth_dir / "mate.json").write_text('{"type": "claude",}')
        requests = self.start_management(
            [{"name": "mate.json", "provider": "claude", "email": "mate@x.test", "priority": 1}]
        )
        status, _, error = self.prioritize_at(NOW, {})
        self.assertEqual((status, requests), (1, []))
        self.assertIn('Cannot reserve "mate.json"', error)

    def test_a_credential_changed_into_one_prioritize_cannot_manage_is_not_written(self):
        self.own_accounts.write_text("me@x.test\n")
        broken = self.account(email="broken@x.test", excluded_models=RESERVE)
        linked = self.account(email="linked@x.test", excluded_models=RESERVE)
        requests = self.start_management(
            [{"name": path.name, "priority": 0} for path in (broken, linked)]
        )

        def response(provider, auth, timeout):
            if auth["email"] == "broken@x.test":
                broken.write_text('{"type": "claude",}')
            else:
                target = self.auth_dir / "shared"
                target.write_text(linked.read_text())
                linked.unlink()
                linked.symlink_to(target)
            return claude_usage(90, 50)

        with (
            patch.object(quota.time, "time", return_value=NOW),
            patch.object(quota, "fetch_usage", side_effect=response),
        ):
            status, _, error = self.run_cli("prioritize")
        self.assertEqual((status, requests), (1, []))
        self.assertIn(f'Cannot update "{broken.name}"', error)
        self.assertIn(f'Cannot update "{linked.name}"', error)

    def test_a_priority_write_keeps_an_exclusion_set_by_hand_during_the_fetch(self):
        self.own_accounts.write_text("me@x.test\n")
        path = self.account(email="mate@x.test", excluded_models=RESERVE)
        requests = self.start_management([{"name": path.name, "priority": 0}])

        def response(provider, auth, timeout):
            edited = {**json.loads(path.read_text()), "excluded_models": RESERVE + ["*"]}
            path.write_text(json.dumps(edited))
            return claude_usage(90, 50)

        with (
            patch.object(quota.time, "time", return_value=NOW),
            patch.object(quota, "fetch_usage", side_effect=response),
        ):
            status, _, _ = self.run_cli("prioritize")
        self.assertEqual(
            (status, requests, json.loads(path.read_text())["excluded_models"]),
            (0, [(FIELDS, {"name": path.name, "priority": 1, "excluded_models": ["*"]})], ["*"]),
        )

    def test_nested_teammates_in_the_proxy_are_reported_without_writes(self):
        self.own_accounts.write_text("me@x.test\n")
        (self.auth_dir / "team").mkdir()
        (self.auth_dir / "team/mate.json").write_text(
            json.dumps({"type": "claude", "email": "me@x.test", "excluded_models": RESERVE})
        )
        requests = self.start_management(
            [
                {"name": "team/mate.json", "provider": "claude", "email": "mate@x.test"},
                {"name": "team/removed.json", "provider": "claude"},
                {"name": "team/own.json", "provider": "claude", "email": "ME@x.test"},
                {"name": "team/codex.json", "provider": "codex"},
            ]
        )
        with patch.object(quota, "fetch_usage", return_value=claude_usage(10, 50)) as fetch:
            status, _, error = self.run_cli("prioritize")
            fetch.assert_not_called()
        self.assertEqual((status, requests), (1, []))
        self.assertIn('Cannot reserve "team/mate.json"', error)
        self.assertIn('Cannot reserve "team/removed.json"', error)
        self.assertIn("at the top level of the auth directory", error)
        self.assertNotIn("team/own.json", error)
        self.assertNotIn("team/codex.json", error)
        with patch.object(quota, "fetch_usage", return_value=claude_usage(10, 50)):
            status, output, _ = self.run_cli("--json")
        self.assertEqual(status, 0)
        (account,) = json.loads(output)
        self.assertEqual((account["file"], account["reserved"]), ("team/mate.json", False))

    def test_nested_edits_survive_while_top_level_prioritization_continues(self):
        self.own_accounts.write_text("me@x.test\n")
        top_level = self.account(email="me@x.test").name
        (self.auth_dir / "team").mkdir()
        edits = {
            "team/exclusions.json": {"excluded_models": RESERVE + ["*"]},
            "team/replacement.json": {"email": "new@x.test", "access_token": "new-token"},
        }
        initial = {"type": "claude", "email": "mate@x.test", "excluded_models": RESERVE}
        for name in edits:
            (self.auth_dir / name).write_text(json.dumps(initial))
        requests = self.start_management(
            [{"name": top_level, "priority": 0}]
            + [
                {"name": name, "provider": "claude", "email": "mate@x.test"}
                for name in edits
            ]
        )

        def response(provider, auth, timeout):
            if auth["email"] == "me@x.test":
                for name, changes in edits.items():
                    (self.auth_dir / name).write_text(json.dumps({**initial, **changes}))
            return claude_usage(90, 50)

        with (
            patch.object(quota.time, "time", return_value=NOW),
            patch.object(quota, "fetch_usage", side_effect=response),
        ):
            status, _, _ = self.run_cli("prioritize")
        self.assertEqual((status, requests), (1, [(FIELDS, {"name": top_level, "priority": 1})]))
        for name, changes in edits.items():
            self.assertEqual(json.loads((self.auth_dir / name).read_text()), {**initial, **changes})

    def test_a_credential_is_written_under_the_name_cliproxyapi_lists(self):
        self.own_accounts.write_text("me@x.test\n")
        names = {".json": "bare@x.test", "mate\u00a0backup.json": "space@x.test"}
        for name, email in names.items():
            (self.auth_dir / name).write_text(json.dumps({"type": "claude", "email": email}))
        requests = self.start_management([{"name": name, "priority": 0} for name in names])
        status, _, _ = self.prioritize_at(
            NOW, {email: claude_usage(10, 50) for email in names.values()}
        )
        self.assertEqual(
            (status, excluded_writes(requests)),
            (0, [(name, RESERVE) for name in sorted(names)]),
        )

    def test_a_teammate_named_like_another_credential_after_trimming_is_reported(self):
        self.own_accounts.write_text("me@x.test\n")
        for name, email in {"mate.json": "me@x.test", " mate.json": "mate@x.test"}.items():
            (self.auth_dir / name).write_text(json.dumps({"type": "claude", "email": email}))
        requests = self.start_management(
            [
                {"id": file_id, "name": "mate.json", "provider": "claude",
                 "email": email, "priority": 0}
                for file_id, email in {" mate.json": "mate@x.test", "mate.json": "me@x.test"}.items()
            ]
        )
        status, _, error = self.prioritize_at(
            NOW, {"me@x.test": claude_usage(10, 50), "mate@x.test": claude_usage(10, 50)}
        )
        self.assertEqual((status, excluded_writes(requests)), (1, []))
        self.assertIn('Cannot reserve " mate.json"', error)

    def test_priorities_are_matched_by_the_untrimmed_id(self):
        self.own_accounts.write_text("me@x.test\n")
        accounts = {
            "mate.json": ("mate@x.test", 0, claude_usage(90, 10)),
            "me.json": ("me@x.test", 1, claude_usage(90, 50)),
            " mate.json": ("me@x.test", 2, claude_usage(90, 50)),
        }
        for name, (email, _, _) in accounts.items():
            auth = {"type": "claude", "email": email, "disabled": name == " mate.json"}
            (self.auth_dir / name).write_text(json.dumps(auth))
        requests = self.start_management(
            [
                {"id": name, "name": name.strip(), "provider": "claude",
                 "email": email, "priority": priority}
                for name, (email, priority, _) in accounts.items()
            ]
        )
        status, _, _ = self.prioritize_at(
            NOW, {email: usage for email, _, usage in list(accounts.values())[:2]}
        )
        self.assertEqual(status, 0)
        self.assertIn(("mate.json", 2), priority_writes(requests))

    def test_a_reserve_removed_through_the_management_api_during_the_fetch_is_restored(self):
        self.own_accounts.write_text("me@x.test\n")
        path = self.account(email="mate@x.test", excluded_models=["claude-opus-*"] + RESERVE)
        requests = self.start_management([{"name": path.name, "priority": 0}])

        def response(provider, auth, timeout):
            edited = {**json.loads(path.read_text()), "excluded_models": ["claude-opus-*"]}
            path.write_text(json.dumps(edited))
            return claude_usage(10, 50)

        with (
            patch.object(quota.time, "time", return_value=NOW),
            patch.object(quota, "fetch_usage", side_effect=response),
        ):
            status, _, _ = self.run_cli("prioritize")
        self.assertEqual(
            (status, excluded_writes(requests)),
            (0, [(path.name, ["claude-opus-*"] + RESERVE)]),
        )

    def test_an_own_email_with_padding_stays_open(self):
        self.own_accounts.write_text("me@x.test\n")
        path = self.account(email=" ME@x.test ")
        requests = self.start_management([{"name": path.name, "priority": 0}])
        status, _, _ = self.prioritize_at(NOW, {" ME@x.test ": claude_usage(10, 50)})
        self.assertEqual((status, excluded_writes(requests)), (0, []))

    def test_an_own_email_with_unicode_padding_stays_open(self):
        self.own_accounts.write_text("me@x.test\n")
        path = self.account(email="\u00a0ME@x.test\t")
        requests = self.start_management([{"name": path.name, "priority": 0}])
        status, _, _ = self.prioritize_at(NOW, {"\u00a0ME@x.test\t": claude_usage(10, 50)})
        self.assertEqual((status, excluded_writes(requests)), (0, []))

    def test_a_teammate_enabled_during_the_fetch_is_kept_out(self):
        self.own_accounts.write_text("me@x.test\n")
        own = self.account(email="me@x.test")
        mate = self.account(email="mate@x.test", disabled=True)
        requests = self.start_management(
            [{"name": path.name, "priority": 0} for path in (own, mate)]
        )

        mate_read = threading.Event()
        account_status = quota.account_status

        def read(auth_dir, file, *args):
            status = account_status(auth_dir, file, *args)
            if file == mate.name:
                mate_read.set()
            return status

        def response(provider, auth, timeout):
            mate_read.wait(5)
            mate.write_text(json.dumps({**json.loads(mate.read_text()), "disabled": False}))
            return claude_usage(90, 50)

        with (
            patch.object(quota.time, "time", return_value=NOW),
            patch.object(quota, "account_status", side_effect=read),
            patch.object(quota, "fetch_usage", side_effect=response),
        ):
            status, _, _ = self.run_cli("prioritize")
        self.assertEqual((status, excluded_writes(requests)), (0, [(mate.name, RESERVE)]))

    def test_a_teammate_without_an_email_is_not_owned_by_its_file_name(self):
        self.own_accounts.write_text("me@x.test\n")
        path = self.auth_dir / "me@x.test.json"
        path.write_text(json.dumps({"type": "claude", "email": " "}))
        requests = self.start_management([{"name": path.name, "priority": 0}])
        with (
            patch.object(quota.time, "time", return_value=NOW),
            patch.object(quota, "fetch_usage", return_value=claude_usage(10, 50)),
        ):
            status, _, _ = self.run_cli("prioritize")
        self.assertEqual((status, excluded_writes(requests)), (0, [(path.name, RESERVE)]))

    def test_a_case_only_email_edit_during_the_fetch_still_closes_a_teammate(self):
        self.own_accounts.write_text("me@x.test\n")
        path = self.account(email="mate@x.test")
        requests = self.start_management([{"name": path.name, "priority": 1}])

        def response(provider, auth, timeout):
            path.write_text(json.dumps({**json.loads(path.read_text()), "email": "mate@X.TEST"}))
            return claude_usage(10, 50)

        with (
            patch.object(quota.time, "time", return_value=NOW),
            patch.object(quota, "fetch_usage", side_effect=response),
        ):
            status, _, _ = self.run_cli("prioritize")
        self.assertEqual((status, excluded_writes(requests)), (0, [(path.name, RESERVE)]))

    def test_an_unreadable_subdirectory_does_not_stop_reserving_top_level_teammates(self):
        self.own_accounts.write_text("me@x.test\n")
        backup = self.auth_dir / "backup"
        backup.mkdir()
        backup.chmod(0)
        self.addCleanup(backup.chmod, 0o700)
        path = self.account(email="mate@x.test")
        requests = self.start_management([{"name": path.name, "priority": 0}])
        status, _, _ = self.prioritize_at(NOW, {"mate@x.test": claude_usage(10, 50)})
        self.assertEqual((status, excluded_writes(requests)), (0, [(path.name, RESERVE)]))

    def test_a_replaced_credential_is_not_reopened_on_the_old_cached_reading(self):
        self.own_accounts.write_text("me@x.test\n")
        path = self.auth_dir / "shared.json"
        old = {"type": "claude", "access_token": "token-a", "excluded_models": RESERVE}
        path.write_text(json.dumps(old))
        usage = {"token-a": claude_usage(90, 50), "token-b": claude_usage(10, 50)}

        def response(provider, auth, timeout):
            return usage[auth["access_token"]]

        with (
            patch.object(quota.time, "time", return_value=NOW),
            patch.object(quota, "fetch_usage", side_effect=response),
        ):
            self.run_cli("--json")
            path.write_text(json.dumps({**old, "access_token": "token-b"}))
            requests = self.start_management([{"name": path.name, "priority": 0}])
            status, _, _ = self.run_cli("prioritize")
        self.assertEqual((status, excluded_writes(requests)), (0, []))

    def test_an_unreadable_subdirectory_does_not_hide_readable_accounts(self):
        backup = self.auth_dir / "backup"
        backup.mkdir()
        backup.chmod(0)
        self.addCleanup(backup.chmod, 0o700)
        self.account(email="mate@x.test")
        with patch.object(quota, "fetch_usage", return_value=claude_usage(90, 50)):
            status, output, _ = self.run_cli("--json")
        self.assertEqual((status, [a["email"] for a in json.loads(output)]), (0, ["mate@x.test"]))

    def test_a_credential_replaced_during_the_fetch_is_not_reopened(self):
        self.own_accounts.write_text("me@x.test\n")
        path = self.auth_dir / "shared.json"
        old = {"type": "claude", "access_token": "token-a", "excluded_models": RESERVE}
        path.write_text(json.dumps(old))
        requests = self.start_management([{"name": path.name, "priority": 0}])

        def response(provider, auth, timeout):
            path.write_text(json.dumps({**old, "access_token": "token-b"}))
            return claude_usage(90, 50)

        with (
            patch.object(quota.time, "time", return_value=NOW),
            patch.object(quota, "fetch_usage", side_effect=response),
        ):
            status, _, _ = self.run_cli("prioritize")
        self.assertEqual((status, excluded_writes(requests)), (0, []))

    def test_a_replaced_credential_does_not_show_the_old_cached_quota(self):
        path = self.auth_dir / "shared.json"
        old = {"type": "claude", "access_token": "token-a"}
        path.write_text(json.dumps(old))

        def response(provider, auth, timeout):
            if auth["access_token"] == "token-b":
                raise urllib.error.HTTPError("http://usage.test", 401, "No", {}, io.BytesIO())
            return claude_usage(90, 50)

        with patch.object(quota, "fetch_usage", side_effect=response):
            self.run_cli("--json")
            path.write_text(json.dumps({**old, "access_token": "token-b"}))
            status, output, _ = self.run_cli("--json")
        self.assertEqual((status, json.loads(output)[0]["windows"]), (1, []))

    def test_a_file_name_with_a_line_break_is_shown_on_one_line(self):
        (self.auth_dir / "two\nlines.json").write_text("{")
        status, output, _ = self.run_cli()
        self.assertEqual(status, 1)
        self.assertIn("two?lines.json", output)

    def test_a_reading_for_a_replaced_credential_does_not_reopen_it(self):
        self.own_accounts.write_text("me@x.test\n")
        path = self.account(email="old@x.test", excluded_models=RESERVE)
        requests = self.start_management([{"name": path.name, "priority": 0}])

        def response(provider, auth, timeout):
            replaced = {**json.loads(path.read_text()), "email": "new@x.test"}
            path.write_text(json.dumps(replaced))
            return claude_usage(90, 50)

        with (
            patch.object(quota.time, "time", return_value=NOW),
            patch.object(quota, "fetch_usage", side_effect=response),
        ):
            status, _, _ = self.run_cli("prioritize")
        self.assertEqual((status, excluded_writes(requests)), (0, []))

    def test_a_hand_set_exclusion_made_during_the_fetch_survives_a_reopen(self):
        self.own_accounts.write_text("me@x.test\n")
        hand_set = {
            "added@x.test": (RESERVE, RESERVE + ["*"]),
            "replaced@x.test": (["claude-opus-*"] + RESERVE, ["*"]),
        }
        paths = {
            email: self.account(email=email, excluded_models=before)
            for email, (before, _) in hand_set.items()
        }
        requests = self.start_management(
            [{"name": path.name, "priority": 0} for path in paths.values()]
        )

        def response(provider, auth, timeout):
            path = paths[auth["email"]]
            edited = {**json.loads(path.read_text()), "excluded_models": hand_set[auth["email"]][1]}
            path.write_text(json.dumps(edited))
            return claude_usage(90, 50)

        with (
            patch.object(quota.time, "time", return_value=NOW),
            patch.object(quota, "fetch_usage", side_effect=response),
        ):
            status, _, _ = self.run_cli("prioritize")
        self.assertEqual(
            (
                status,
                excluded_writes(requests),
                [json.loads(path.read_text())["excluded_models"] for path in paths.values()],
            ),
            (0, [(path.name, ["*"]) for path in paths.values()], [["*"], ["*"]]),
        )

    def test_only_the_reserve_pair_in_order_lets_an_account_back_in(self):
        self.own_accounts.write_text("me@x.test\n")
        hand_set = {
            "swapped@x.test": ["cliproxy-quota-reserve", "*"],
            "apart@x.test": ["*", "claude-opus-*", "cliproxy-quota-reserve"],
            "lone@x.test": ["cliproxy-quota-reserve"],
        }
        names = [
            self.account(email=email, excluded_models=models).name
            for email, models in hand_set.items()
        ]
        requests = self.start_management([{"name": name, "priority": 0} for name in names])
        status, _, _ = self.prioritize_at(
            NOW, {email: claude_usage(90, 50) for email in hand_set}
        )
        with patch.object(quota.time, "time", return_value=NOW):
            _, output, _ = self.run_cli("--json")
        self.assertEqual(
            (status, excluded_writes(requests)),
            (0, []),
        )
        self.assertEqual(
            [account["reserved"] for account in json.loads(output)],
            [False, False, False],
        )

    def test_a_teammate_account_closes_and_reopens_around_hand_set_exclusions(self):
        self.own_accounts.write_text("me@x.test\n")
        hand_set = {
            "opus@x.test": {"excluded_models": ["claude-opus-*"]},
            "empty@x.test": {"excluded_models": []},
            "alias@x.test": {"excluded-models": ["claude-opus-*"]},
            "text@x.test": {"excluded_models": "claude-opus-*"},
        }
        names = [
            self.account(email=email, **fields).name for email, fields in hand_set.items()
        ]
        requests = self.start_management([{"name": name, "priority": 0} for name in names])
        spent, _, _ = self.prioritize_at(
            NOW, {email: claude_usage(0, 50) for email in hand_set}
        )
        healthy, _, _ = self.prioritize_at(
            NOW + 600, {email: claude_usage(90, 50) for email in hand_set}
        )
        opus = ["claude-opus-*"]
        self.assertEqual(
            (spent, healthy, excluded_writes(requests)),
            (
                0,
                0,
                [
                    (names[0], opus + RESERVE),
                    (names[1], RESERVE),
                    (names[2], opus + RESERVE),
                    (names[3], opus + RESERVE),
                    (names[0], opus),
                    (names[1], None),
                    (names[2], opus),
                    (names[3], opus),
                ],
            ),
        )

    def test_closing_keeps_only_the_hand_set_entries_cliproxyapi_accepts(self):
        self.own_accounts.write_text("me@x.test\n")
        mate = self.account(
            email="mate@x.test", excluded_models=["claude-opus-*", None]
        ).name
        requests = self.start_management([{"name": mate, "priority": 0}])
        status, _, _ = self.prioritize_at(NOW, {"mate@x.test": claude_usage(0, 50)})
        self.assertEqual(
            (status, excluded_writes(requests)),
            (0, [(mate, ["claude-opus-*", *RESERVE])]),
        )

    def test_reopening_keeps_hand_set_entries_added_while_closed(self):
        self.own_accounts.write_text("me@x.test\n")
        added = {"opus@x.test": [*RESERVE, "claude-opus-*"], "every@x.test": [*RESERVE, "*"]}
        names = [
            self.account(email=email, excluded_models=models).name
            for email, models in added.items()
        ]
        requests = self.start_management([{"name": name, "priority": 0} for name in names])
        status, _, _ = self.prioritize_at(
            NOW, {email: claude_usage(90, 50) for email in added}
        )
        self.assertEqual(
            (status, excluded_writes(requests)),
            (0, [(names[0], ["claude-opus-*"]), (names[1], ["*"])]),
        )

    def test_prioritize_sends_no_status_or_excluded_models_to_disabled_accounts(self):
        self.own_accounts.write_text("me@x.test\n")
        off_mate = self.account(email="off-mate@x.test", disabled=True).name
        off_me = self.account(
            email="me@x.test", disabled=True, excluded_models=RESERVE
        ).name
        mate = self.account(email="mate@x.test").name
        requests = self.start_management(
            [{"name": name, "priority": 2} for name in (off_mate, off_me, mate)]
        )
        status, _, _ = self.prioritize_at(NOW, {"mate@x.test": claude_usage(0, 50)})
        self.assertEqual(status, 0)
        self.assertEqual([path for path, _ in requests if path != FIELDS], [])
        self.assertEqual(excluded_writes(requests), [(mate, RESERVE)])
        self.assertEqual(
            [body for _, body in requests if body["name"] != mate],
            [{"name": off_mate, "priority": 0}, {"name": off_me, "priority": 0}],
        )

    def test_a_failed_write_does_not_stop_the_other_accounts(self):
        self.own_accounts.write_text("me@x.test\n")
        gone_spent, spent = [
            self.account(email=email).name for email in ("a@x.test", "b@x.test")
        ]
        gone_healthy, healthy = [
            self.account(email=email, excluded_models=RESERVE).name
            for email in ("c@x.test", "d@x.test")
        ]
        names = (gone_spent, spent, gone_healthy, healthy)
        requests = self.start_management(
            [{"name": name, "priority": 0} for name in names],
            gone={gone_spent, gone_healthy},
        )
        usage = {
            "a@x.test": claude_usage(0, 50),
            "b@x.test": claude_usage(0, 50),
            "c@x.test": claude_usage(90, 50),
            "d@x.test": claude_usage(90, 60),
        }
        status, output, error = self.prioritize_at(NOW, usage)
        self.assertEqual(
            (status, requests),
            (
                1,
                [
                    (FIELDS, {"name": gone_spent, "excluded_models": RESERVE}),
                    (FIELDS, {"name": spent, "excluded_models": RESERVE}),
                    (FIELDS, {"name": gone_healthy, "priority": 2, "excluded_models": None}),
                    (FIELDS, {"name": healthy, "priority": 1, "excluded_models": None}),
                ],
            ),
        )
        self.assertEqual(
            error,
            f'{gone_spent}  excluded_models [] -> ["*", "cliproxy-quota-reserve"] failed: '
            "HTTP Error 404: Not Found\n"
            f"{gone_healthy}  priority 0 -> 2 failed: HTTP Error 404: Not Found\n"
            f'{gone_healthy}  excluded_models ["*", "cliproxy-quota-reserve"] -> null failed: '
            "HTTP Error 404: Not Found\n",
        )

    def test_a_second_run_without_changes_sends_nothing(self):
        self.own_accounts.write_text("me@x.test\n")
        emails = ("me@x.test", "spent@x.test", "recovered@x.test")
        me, spent = [self.account(email=email).name for email in emails[:2]]
        recovered = self.account(email=emails[2], excluded_models=RESERVE).name
        requests = self.start_management(
            [{"name": name, "priority": 0} for name in (me, spent, recovered)]
        )
        usage = {
            "me@x.test": claude_usage(90, 50),
            "spent@x.test": claude_usage(0, 50),
            "recovered@x.test": claude_usage(90, 60),
        }
        first = self.prioritize_at(NOW, usage)
        sent = list(requests)
        second = self.prioritize_at(NOW + 60, usage)
        self.assertEqual((first[0], len(sent)), (0, 3))
        self.assertEqual((second, requests), ((0, "", ""), sent))

    def test_prioritize_keeps_each_teammate_their_share_of_weekly_quota(self):
        self.own_accounts.write_text("own-a@x.test\nown-b@x.test\n")
        left = {
            "mate-a@x.test": (0, 31.0),
            "mate-b@x.test": (83, 63.8),
            "mate-c@x.test": (95, 73.9),
            "mate-d@x.test": (100, 78.0),
            "own-a@x.test": (0, 58.4),
            "own-b@x.test": (99, 83.4),
        }
        names = {email: self.account(email=email).name for email in left}
        requests = self.start_management(
            [{"name": name, "priority": 0} for name in names.values()]
        )
        usage = {email: claude_usage(*reading) for email, reading in left.items()}
        status, _, _ = self.prioritize_at(NOW, usage)
        self.assertEqual(
            (status, requests),
            (
                0,
                [
                    (FIELDS, {"name": names["mate-a@x.test"], "excluded_models": RESERVE}),
                    (FIELDS, {"name": names["mate-b@x.test"], "priority": 4}),
                    (FIELDS, {"name": names["mate-c@x.test"], "priority": 3}),
                    (FIELDS, {"name": names["mate-d@x.test"], "priority": 2}),
                    (FIELDS, {"name": names["own-b@x.test"], "priority": 1}),
                ],
            ),
        )

    def test_prioritize_fails_on_a_wrong_or_missing_management_key(self):
        self.account(email="only@x.test")
        requests = self.start_management([])
        quota.MANAGEMENT_KEY.write_text("wrong")
        status, _, error = self.run_cli("prioritize")
        self.assertEqual((status, requests), (1, []))
        self.assertIn("Cannot set priorities: HTTP Error 401", error)
        quota.MANAGEMENT_KEY.unlink()
        status, _, error = self.run_cli("prioritize")
        self.assertEqual(status, 1)
        self.assertIn("management-key", error)

    def test_prioritize_rejects_a_malformed_management_response(self):
        self.start_management([])
        for response in ([], {"files": "x"}, {"files": [{"priority": 1}]}):
            with self.subTest(response=response), patch.object(
                quota, "management_request", return_value=response
            ):
                status, _, error = self.run_cli("prioritize")
                self.assertEqual(status, 1)
                self.assertIn("invalid management response", error)

    def test_accounts_are_listed_in_place_in_line(self):
        for provider, email, priority in (
            ("claude", "first@x.test", "10"),
            ("claude", "unranked@x.test", 0),
            ("claude", "second@x.test", 2),
            ("codex", "codex@x.test", 9),
        ):
            self.account(provider, email=email, priority=priority)
        self.account(email="disabled@x.test", priority=20, disabled=True)
        (self.auth_dir / "broken.json").write_text("{")
        usage = {
            "seven_day": {"utilization": 40},
            "rate_limit": {"primary_window": {"used_percent": 1}},
        }
        with patch.object(quota, "fetch_usage", return_value=usage):
            _, output, _ = self.run_cli()
            _, json_output, _ = self.run_cli("--json")
        self.assertTrue(output.endswith("broken.json\n  Cannot read credential JSON\n"))
        self.assertEqual(
            [line for line in output.splitlines() if line.startswith(("claude", "codex"))],
            [
                "claude  first@x.test  priority 1",
                "claude  second@x.test  priority 2",
                "claude  unranked@x.test",
                "claude  disabled@x.test",
                "codex  codex@x.test  priority 1",
            ],
        )
        self.assertEqual(
            [account["priority"] for account in json.loads(json_output)],
            [1, 2, None, None, 1, None],
        )
        with patch.object(quota, "fetch_usage", return_value=usage):
            _, served = self.get_served(self.auth_dir)
        served = json.loads(served)
        self.assertEqual(
            served,
            [
                {key: account[key] for key in served[0]}
                for account in json.loads(json_output)
            ],
        )

    def test_credential_priority_reads_what_cliproxyapi_accepts(self):
        for value, expected in (
            (3, 3),
            (" +7 ", 7),
            (2.9, 2),
            (0, None),
            (-1, None),
            ("5_0", None),
            ("\x1c10", None),
            ("\u00a0 4\n", 4),
            ("x", None),
            ("9223372036854775807", 2**63 - 1),
            ("9223372036854775808", None),
            (1e19, None),
            ("9" * 5000, None),
            ("0" * 19 + "1", 1),
            ("+" + "0" * 5000 + "7", 7),
            ("-" + "0" * 30 + "7", None),
            (10**400, None),
            (9007199254740993, 9007199254740992),
            ("9007199254740993", 9007199254740993),
            (True, None),
            (None, None),
            (float("inf"), None),
        ):
            with self.subTest(value=value):
                self.assertEqual(quota.credential_priority(value), expected)

if __name__ == "__main__":
    unittest.main()
