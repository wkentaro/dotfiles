import io
import json
import shutil
import tempfile
import time
import unittest
import urllib.error
from contextlib import redirect_stderr, redirect_stdout
from datetime import datetime, timezone
from http.client import BadStatusLine, IncompleteRead
from importlib.machinery import SourceFileLoader
from importlib.util import module_from_spec, spec_from_loader
from pathlib import Path
from unittest.mock import patch

SCRIPT = Path(__file__).resolve().parents[1] / "local/bin/cliproxy-quota"
LOADER = SourceFileLoader("cliproxy_quota", str(SCRIPT))
quota = module_from_spec(spec_from_loader(LOADER.name, LOADER))
with patch("sys.dont_write_bytecode", True):
    LOADER.exec_module(quota)


def usage_lines(provider, data):
    return quota.window_lines(quota.quota_windows(provider, data))


class QuotaTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.auth_dir = Path(self.directory.name) / "auth"
        self.auth_dir.mkdir()
        self.cache_dir = Path(self.directory.name) / "cache"
        cache_patch = patch.object(quota, "CACHE_DIR", self.cache_dir)
        cache_patch.start()
        self.addCleanup(cache_patch.stop)

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
            status = quota.main(["--auth-dir", str(self.auth_dir), *args])
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
        self.assertEqual(len(lines), 3)
        self.assertIn("not reported", lines[0])
        self.assertIn("[####------]  37.5% left", lines[1])
        self.assertIn("sonnet weekly", lines[2])
        self.assertIn("100.0% left", lines[2])
        self.assertIn("reset not reported", lines[2])

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
        self.assertIn("[----------]   0.0% left", lines[1])

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
                self.assertIn("not reported", quota.window_line("weekly", value, None))

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

    def test_reset_time_converts_iso_and_epoch_to_local_timezone(self):
        expected = "resets " + datetime.fromtimestamp(
            1795484535, timezone.utc
        ).astimezone().strftime("%Y-%m-%d %H:%M %Z")
        self.assertEqual(quota.reset_text(1795484535), expected)
        self.assertEqual(quota.reset_text("2026-11-24T01:42:15Z"), expected)
        for value in (None, True, {}, "bad", "2026-11-19T07:00:00", 10**100):
            self.assertEqual(quota.reset_text(value), "reset not reported")

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
            output, failed = quota.account_report(path, 1)
        self.assertTrue(failed)
        self.assertIn("Invalid credentials or quota response", output)
        with patch.object(quota, "fetch_usage", return_value={}):
            status, output, error = self.run_cli()
        self.assertEqual(status, 1)
        self.assertIn("Cannot read credential JSON", output)
        self.assertIn("claude  member@example.test", output)
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
            with (
                self.subTest(error=error),
                patch.object(quota, "fetch_usage", side_effect=error),
            ):
                output, failed = quota.account_report(path, 1)
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
            output, failed = quota.account_report(path, 1)
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
                output, failed = quota.account_report(path, 1)
            self.assertTrue(failed)
            self.assertIn(f"HTTP 429; retry in {expected}", output)

    def report_at(self, path, now, response=None):
        response = response or {"seven_day": {"utilization": 0}}
        with (
            patch.object(quota.time, "time", return_value=now),
            patch.object(quota, "fetch_usage", return_value=response) as fetch,
        ):
            output, failed = quota.account_report(path, 1)
        return output, failed, fetch.call_count

    def cache_files(self, suffix):
        return list(self.cache_dir.glob("*" + suffix))

    def test_model_specific_claude_window_alone_is_reported(self):
        lines = usage_lines("claude", {"seven_day_opus": {"utilization": 20}})
        self.assertIn("opus weekly", lines[2])
        self.assertIn("80.0% left", lines[2])

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

    def test_rate_limit_blocks_requests_and_keeps_showing_cached_quota(self):
        path = self.account()
        now = time.time()
        self.report_at(path, now, {"seven_day": {"utilization": 40}})
        with (
            patch.object(quota.time, "time", return_value=now + 301),
            self.rate_limit({"Retry-After": "120"}),
        ):
            output, failed = quota.account_report(path, 1)
        self.assertTrue(failed)
        self.assertIn("60.0% left", output)
        self.assertIn("HTTP 429; retry in 2 min", output)
        output, failed, calls = self.report_at(path, now + 400)
        self.assertEqual(calls, 0)
        self.assertTrue(failed)
        self.assertIn("as of", output)
        self.assertIn("60.0% left", output)
        self.assertIn("HTTP 429; retry in 1 min", output)
        output, failed, calls = self.report_at(path, now + 422)
        self.assertEqual(calls, 1)
        self.assertFalse(failed)
        self.assertIn("100.0% left", output)

    def test_other_failures_keep_showing_cached_quota(self):
        path = self.account()
        now = time.time()
        self.report_at(path, now, {"seven_day": {"utilization": 40}})
        with (
            patch.object(quota.time, "time", return_value=now + 301),
            patch.object(quota, "fetch_usage", side_effect=OSError("secret")),
        ):
            output, failed = quota.account_report(path, 1)
        self.assertTrue(failed)
        self.assertIn("as of", output)
        self.assertIn("60.0% left", output)
        self.assertIn("Network request failed", output)

    def test_unreadable_or_invalid_cache_is_ignored(self):
        now = time.time()
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
            (".retry", "true"),
            (".retry", json.dumps(now + 10**9)),
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
        self.cache_dir.mkdir()
        self.cache_dir.chmod(0o500)
        self.addCleanup(self.cache_dir.chmod, 0o700)
        output, failed, calls = self.report_at(self.account(), time.time())
        self.assertEqual(calls, 1)
        self.assertFalse(failed)
        self.assertIn("100.0% left", output)


if __name__ == "__main__":
    unittest.main()
