---
name: v
description: Signals that this message was dictated by voice, so technical terms may be mis-transcribed. Use when the user prefixes a dictated message with /v.
disable-model-invocation: true
argument-hint: "<your dictated message>"
---

# Voice dictation

The text in `$ARGUMENTS` came from voice dictation, **for this turn only**. Speech-to-text
garbles terms that have no phonetic redundancy: repo and project names, CLI tool names,
command flags, file paths, identifiers, and acronyms. Ordinary prose is usually fine; the
precise, consequential nouns are where it breaks.

Apply this protocol to the dictated message, then carry out the request. Do not apply it to
later messages unless they also begin with `/v`.

## 1. Readback

Open your reply with a single one-line readback that paraphrases the request with the
technical terms resolved. Put terms you resolved or are assuming in `code font` so the user
can catch a wrong guess at a glance.

> Heard: deploy the `widget-sync` service after running `terraform apply`.

## 2. Ask vs. flag

For each suspect term that is **both** likely mis-transcribed **and** consequential
(getting it wrong sends you down the wrong path):

- **Can confidently resolve it?** State the correction you're assuming (in the readback) and
  proceed. Example: "boxer tek you bear netty's" → assume `kubectl get pods` and move on.
- **Cannot confidently resolve it?** Stop and ask before doing anything that depends on it.
  Never act on a term you couldn't confidently resolve, and never mutate state on one.

Do not interrogate ordinary words. Only the high-cost-if-wrong terms warrant a question.

## 3. Don't freeze on a partial blocker

If the ambiguity blocks only part of the task, say what you can safely start and what you
need from the user, rather than halting the whole thing on one trivial clarification.
