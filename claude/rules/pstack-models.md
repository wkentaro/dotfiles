# pstack model configuration. One line per role. Delete a line to fall back to the skill default.
# `inherit-parent` or `auto` as a value: the role runs on the parent chat model (omit Task `model` and spawn `generalPurpose`). Alias entries in a panel list still count toward its fan-out.
# `sol` or `astra` as a value: spawn that subagent type (`subagent_type: "sol"` or `"astra"`), omit `model`, and pass the effort. Both are GPT, so they judge for a Claude parent and `opus` judges for a GPT parent.
# defaults: 3
# budget: large (xhigh)
feature, refactoring: opus xhigh
bug-fix: opus xhigh
perf-issue: opus xhigh
hillclimb: opus xhigh
judgment and prose: opus xhigh
arena runners: fable high, opus high, sol high
arena cross-judge pool: sol high, astra high, opus high
swarm workers: opus high
interrogate reviewers: fable high, opus high, astra high
