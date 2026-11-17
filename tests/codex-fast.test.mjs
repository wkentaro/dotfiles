import assert from "node:assert/strict";
import { test } from "node:test";
import register from "../pi/agent/extensions/codex-fast.ts";

function createExtension(branch = [], api = "openai-codex-responses", hasUI = true) {
  const handlers = new Map();
  const commands = new Map();
  const statuses = new Map();
  const notifications = [];
  const ctx = {
    hasUI,
    model: { api },
    sessionManager: { getBranch: () => branch },
    ui: {
      setStatus: (key, value) => statuses.set(key, value),
      notify: (...args) => notifications.push(args),
    },
  };
  register({
    on: (event, handler) => handlers.set(event, handler),
    registerCommand: (name, command) => commands.set(name, command),
    appendEntry: (customType, data) => branch.push({ type: "custom", customType, data }),
  });
  const emit = (event, data = {}) => handlers.get(event)(data, ctx);
  emit("session_start");
  return {
    ctx, branch, statuses, notifications, emit,
    fast: (args = "") => commands.get("fast").handler(args, ctx),
    completions: (prefix) => commands.get("fast").getArgumentCompletions(prefix),
    request: (payload = {}) => emit("before_provider_request", { payload }),
  };
}

test("/fast toggles priority without changing reasoning or input", async () => {
  const ext = createExtension();
  const payload = { model: "gpt-6.1-sol", input: [], reasoning: { effort: "high" } };
  assert.deepEqual(ext.request(payload), { ...payload, service_tier: "default" });
  assert.equal(ext.statuses.get("codex-fast"), "fast off");
  await ext.fast();
  assert.deepEqual(ext.request(payload), { ...payload, service_tier: "priority" });
  assert.equal(ext.statuses.get("codex-fast"), "fast on");
  await ext.fast();
  assert.equal(ext.request().service_tier, "default");
  assert.equal(payload.service_tier, undefined);
});

test("explicit arguments are idempotent and invalid arguments do not change state", async () => {
  const ext = createExtension();
  await ext.fast("off");
  await ext.fast("off");
  assert.equal(ext.request({ service_tier: "priority" }).service_tier, "default");
  const saved = ext.branch.length;
  await ext.fast("invalid");
  assert.equal(ext.branch.length, saved);
  assert.equal(ext.request().service_tier, "default");
  assert.deepEqual(ext.notifications.at(-1), ["Usage: /fast [on|off]", "warning"]);
  await ext.fast(" ON ");
  assert.equal(ext.request().service_tier, "priority");
  assert.deepEqual(ext.completions("o").map(({ value }) => value), ["on", "off"]);
});

test("last valid branch choice survives reload, resume, and fork", async () => {
  const ext = createExtension();
  for (const value of ["on", "off"]) {
    await ext.fast(value);
    ext.branch.push({ type: "custom", customType: "other", data: true });
    ext.branch.push({ type: "custom", customType: "codex-fast", data: "invalid" });
    for (const reason of ["reload", "resume", "fork"]) {
      const restored = createExtension([...ext.branch]);
      restored.emit("session_start", { reason });
      assert.equal(restored.request().service_tier, value === "on" ? "priority" : "default");
      assert.equal(restored.statuses.get("codex-fast"), `fast ${value}`);
    }
  }
});

test("tree navigation and new sessions restore their own state", async () => {
  const ext = createExtension();
  await ext.fast("on");
  const onEntry = ext.branch.pop();
  ext.emit("session_tree");
  assert.equal(ext.request().service_tier, "default");
  ext.branch.push(onEntry);
  ext.emit("session_tree");
  assert.equal(ext.request().service_tier, "priority");
  ext.branch.length = 0;
  ext.emit("session_start", { reason: "new" });
  assert.equal(ext.request().service_tier, "default");
});

test("other APIs and missing models are unchanged and hide the status", async () => {
  const ext = createExtension();
  await ext.fast("off");
  for (const api of ["anthropic-messages", "openai-responses", "openai-completions"]) {
    ext.ctx.model = { api };
    ext.emit("model_select");
    assert.equal(ext.request(), undefined);
    assert.equal(ext.statuses.get("codex-fast"), undefined);
  }
  await ext.fast("on");
  assert.match(ext.notifications.at(-1)[0], /Applies to Codex models/);
  ext.ctx.model = undefined;
  ext.emit("model_select");
  assert.equal(ext.request(), undefined);
  ext.ctx.model = { api: "openai-codex-responses" };
  ext.emit("model_select");
  assert.equal(ext.statuses.get("codex-fast"), "fast on");
});

test("headless requests and commands do not require UI", async () => {
  const ext = createExtension([], "openai-codex-responses", false);
  ext.ctx.ui = undefined;
  await ext.fast("off");
  await ext.fast("invalid");
  assert.equal(ext.request().service_tier, "default");
  assert.equal(ext.branch.length, 1);
});
