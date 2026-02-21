# Claude Code Instructions — AI Workbench

## First Steps Every Session

1. Read `AGENTS.md` — orchestration rules, delegation contracts, canonical governance.
2. Read the project `PROJECT_CONTEXT.md` (use `project_templates/PROJECT_CONTEXT.md` if starting fresh).
3. Identify the lead skill for the task from `skills/`.

## Skill Usage

Reference the relevant `SKILL.md` alongside `AGENTS.md` and `PROJECT_CONTEXT.md`.

Standard prompt pattern:
```text
Follow AGENTS.md, skills/<ROLE_SKILL>/SKILL.md, and PROJECT_CONTEXT.md.
Goal: ...
Deliverables: ...
Constraints: ...
```

## Prompt Verification

For prompts that are long, complex, unclear, or underscoped:
- Surface what is ambiguous or missing before proceeding.
- Propose a refined version of the prompt.
- Wait for approval before executing.

Do not silently interpret vague instructions. Make the gap visible first.

## Session Cut Signals

Produce a handoff packet (`project_templates/HANDOFF_PACKET.md`) and end the session when:
- Phase shifts: planning → implementation → QA → review.
- Scope changed significantly mid-session.
- Responses becoming vague or repetitive (context degrading).
- A blocker cannot be resolved with the current approach.

## Subagent Use

Use Task tool subagents for parallelizable or isolated work:
- `Explore`: codebase research, file discovery.
- `Bash`: git operations, shell commands.
- `Plan`: detailed implementation planning.
- `general-purpose`: complex multi-step research.

Do not duplicate work subagents are already doing.

## Canonical Governance

Canonical — do not modify without passing the promotion gate in `AGENTS.md`:
- `AGENTS.md`, `CLAUDE.md`, `skills/**`, `project_templates/**` (templates only), `scripts/**`

Project-local — do not promote by default:
- Filled `PROJECT_CONTEXT.md`, `tasks.md`, `worklog.md`, `decisions.md`, `LEARNING_LOG.md`
- Handoff packets, research notes, client/domain-specific constraints
