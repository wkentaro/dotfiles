# Pi Auto Mode smoke check

Use Pi 1.5.2 with the [patched Auto Mode revision](https://github.com/wkentaro/pi-automode/commit/df928a47aa1106522004a80f8f31e4241fd7bf2b).
Follow its [configuration guide](https://github.com/wkentaro/pi-automode/blob/df928a47aa1106522004a80f8f31e4241fd7bf2b/README.md).

Use a temporary `HOME`, `PI_CODING_AGENT_DIR`, and workspace. Load only the
extensions and local test tools needed for the check, not existing Pi settings
or live MCP servers. Live reviewer calls use the selected provider's quota.

- Confirm an approved action runs once on a disposable file.
- Confirm writes to temporary symlinked Pi settings are blocked.
- Confirm cancellation and an unavailable reviewer block execution.
- Repeat harmless calls through codemode, a local MCP test tool, and a subagent;
  verify each call reaches the permission guard.

Remove the temporary profile afterward. This is a manual smoke check, not an
OS sandbox or proof of reviewer accuracy. It does not enable Auto Mode globally.
