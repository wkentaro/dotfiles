import type { ExtensionAPI } from "@earendil-works/pi-coding-agent";

export default function (pi: ExtensionAPI) {
  pi.on("before_provider_request", (event, ctx) => {
    if (ctx.model?.api !== "openai-codex-responses") return;

    return {
      ...(event.payload as Record<string, unknown>),
      service_tier: "priority",
    };
  });
}
