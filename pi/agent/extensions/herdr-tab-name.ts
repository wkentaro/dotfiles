import os from "node:os";
import path from "node:path";
import type { ExtensionAPI } from "@earendil-works/pi-coding-agent";

const script = path.join(os.homedir(), ".claude", "hooks", "herdr-tab-name.sh");

export default function (pi: ExtensionAPI) {
  pi.on("before_agent_start", async (event, ctx) => {
    if (ctx.mode !== "tui" || event.messageType !== "user" || process.env.HERDR_ENV !== "1") return;
    const result = await pi.exec("bash", [script], { timeout: 5000 });
    if (result.code !== 0 || !result.stdout.trim()) return;
    let context: string | undefined;
    try {
      context = JSON.parse(result.stdout).hookSpecificOutput?.additionalContext;
    } catch {
      return;
    }
    if (!context) return;
    return { message: { customType: "herdr-tab-name", content: context, display: false } };
  });
}
