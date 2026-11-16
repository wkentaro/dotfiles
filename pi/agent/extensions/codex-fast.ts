import type { ExtensionAPI, ExtensionContext } from "@earendil-works/pi-coding-agent";

const stateKey = "codex-fast";

export default function (pi: ExtensionAPI) {
  let enabled = true;

  function updateStatus(ctx: ExtensionContext) {
    if (!ctx.hasUI) return;
    ctx.ui.setStatus(
      stateKey,
      ctx.model?.api === "openai-codex-responses" ? `fast ${enabled ? "on" : "off"}` : undefined,
    );
  }

  function restore(ctx: ExtensionContext) {
    enabled = true;
    for (const entry of ctx.sessionManager.getBranch()) {
      if (entry.type === "custom" && entry.customType === stateKey && typeof entry.data === "boolean") {
        enabled = entry.data;
      }
    }
    updateStatus(ctx);
  }

  pi.on("session_start", (_event, ctx) => restore(ctx));
  pi.on("session_tree", (_event, ctx) => restore(ctx));
  pi.on("model_select", (_event, ctx) => updateStatus(ctx));

  pi.registerCommand("fast", {
    description: "Toggle Codex priority processing, which uses more quota: /fast [on|off]",
    getArgumentCompletions: (prefix) =>
      ["on", "off"].filter((value) => value.startsWith(prefix)).map((value) => ({ value, label: value })),
    handler: async (args, ctx) => {
      const value = args.trim().toLowerCase();
      if (value !== "" && value !== "on" && value !== "off") {
        if (ctx.hasUI) ctx.ui.notify("Usage: /fast [on|off]", "warning");
        return;
      }

      enabled = value === "" ? !enabled : value === "on";
      pi.appendEntry(stateKey, enabled);
      updateStatus(ctx);
      if (ctx.hasUI) {
        ctx.ui.notify(
          `Fast mode ${enabled ? "on" : "off"}.${ctx.model?.api === "openai-codex-responses" ? "" : " Applies to Codex models."}`,
          "info",
        );
      }
    },
  });

  pi.on("before_provider_request", (event, ctx) => {
    if (ctx.model?.api !== "openai-codex-responses") return;

    return {
      ...(event.payload as Record<string, unknown>),
      service_tier: enabled ? "priority" : "default",
    };
  });
}
