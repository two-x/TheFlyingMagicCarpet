# Claude Code standing prompts for this repo

Six copy-pasteable prompts. Each one, pasted into a Claude Code session
working in this repo (`CarpetLightCode`), sets up a **standing instruction**
that applies to every future response in that session until you tell it
otherwise — not a one-time task. Paste any subset of the 6; they're
independent of each other (prompts 5 and 6 are a little different — see
their own sections below).

They're written to be self-contained: a fresh Claude session with no other
context should be able to follow one correctly, reproducing the same
content, formatting, and conventions (including terseness and the table of
contents) this repo's docs currently have. You shouldn't need to hand-edit
these prompts yourself over time — if you've also adopted prompt 5, Claude
keeps this file's own instructions in sync as the codebase and conventions
they describe evolve.

## How to use these

1. Pick the prompt(s) you want (see below).
2. Paste the whole fenced block as a single message.
3. Claude keeps doing this at the end of every response for the rest of the
   session (or until you say "stop doing that" / start a new session).

You can paste more than one in the same message if you want several of
these running at once.

---

## 1. Keep README.md's firmware description in sync

Use this if you're actively changing firmware behavior (`src/*.h`,
`src/*.cpp`) and want the repo's `README.md` to stay accurate without a
separate manual doc pass.

```text
Upon each prompt answer from now on, until further notice, for each change
currently staged in git: if any staged file under src/ (firmware source —
.h/.cpp files) changes something README.md documents or should document
(fixture behavior, brightness math, button/config-mode behavior, per-show
logic, AudioBoard's audio-processing pipeline, defaults, persisted Nvm
settings, etc.), update README.md's relevant section(s) to match before
your response ends.

Scope: only touch the section(s) actually affected by what's staged right
now. Don't do a full unrelated audit of the rest of the file each time —
but if you're editing a paragraph and notice it references a stale
API/getter name or wrong default because of an EARLIER unrelated change,
fix that too rather than leaving it wrong next to your own edit.

Formatting/style — match the file's existing conventions exactly:
- One `#` H1 at the very top only. Top-level sections are `##`, subsections
  `###`. Find the right section by its heading text/content, don't assume
  fixed line numbers (the file changes shape over time).
- Prose is dense technical documentation, not marketing copy or a tutorial:
  third person, explains WHY (a design rationale, a bug that motivated a
  fix, a hardware constraint) as often as WHAT. No emoji.
- **Condensed, not padded.** Every fact, number, name, and rationale that's
  currently in a section must survive an edit to it — but cut filler:
  redundant restatement of something the previous sentence already said,
  throat-clearing ("it's worth noting that," "the way this works is"),
  and words that don't carry information. Prefer one tight sentence over
  two loose ones; prefer a semicolon or em-dash clause over "and this
  means that." A rough target: if you can cut 15-25% of a paragraph's
  word count without losing a single fact, do it. This is a standing
  property of the file, not a one-time pass — don't let new prose you add
  regress back to a looser style than what's already there.
- Bugfixes get an inline **Bugfix**: bold lead-in within the relevant
  paragraph, not a separate changelog section.
- Wrap identifiers, filenames, and function/getter names in single
  backticks: `AudioBoard::getBandHitPercent()`, `src/AudioBoard.h`.
- Use **bold** for defaults/key values inline (e.g. "default **68%**"), not
  for whole sentences.
- Use a `| --- |` markdown table for structured index/address/mapping data
  (DMX addresses, fixture directions, config subsettings) rather than a
  bullet list of the same thing.
- Cross-reference other sections in parens, e.g. "(see 'Configuration
  mode' above)" — don't duplicate content that already lives elsewhere in
  the file.
- **Headings stay anchor-friendly.** Avoid packing a heading with an
  em-dash clause, a parenthetical file path, commas, or backticked code —
  e.g. prefer `## Megabars` over `## Megabars — \`megabarLeds[12]\`, DMX
  addr 1, 4, 7...34`. If the dropped detail (a filename, an array name)
  isn't already stated in that section's first sentence or its own table,
  add it there instead of putting it back in the heading. This isn't
  cosmetic: both the GitHub-rendered anchor links below and the
  visualizer's own anchor links depend on headings staying short, plain
  text so their auto-generated slug stays stable and predictable.

**Table of contents — keep it current.** Immediately under the file's
top intro paragraphs (before the first `##` section) is a `## Contents`
section: a markdown list of every `##`/`###` heading in the file, in
order, with `###` headings nested one level under their parent `##` as a
sub-list. Each entry is `[Heading text](#slug)`, where `slug` is the
heading text lowercased, with anything that isn't a letter/digit/space/
hyphen stripped out, then spaces turned into hyphens (e.g. "Encoder and
button UI" → `#encoder-and-button-ui`). This is the same algorithm
GitHub's own heading-anchor renderer uses for plain-text headings (part of
why headings need to stay plain text, per above) — so these links work
natively on github.com with zero extra tooling, AND work inside the
visualizer's "FW readme" popup, which slugifies headings in its own
markdown renderer (`slugify()` in carpet-visualizer.html) using the
identical algorithm. Any time you add, remove, rename, or reorder a
heading, update the `## Contents` list to match in the same edit — a
stale TOC entry (wrong text, dead link, missing entry) is as much a bug
as a wrong technical fact.

IMPORTANT: tools/visualizer/carpet-visualizer.html embeds a byte-for-byte
copy of this file's markdown source inside a
`<script type="text/markdown" id="fwReadmeSrc">` element (so its own "FW
readme" popup can render it without needing a server). Any time you change
README.md under this instruction, ALSO replace that embedded copy with the
new README.md content, verbatim, in the same response — the two must never
drift apart. (This is separate from prompt 2 below, which covers that
file's own *visualizer-specific* help content, not this embedded copy.)

End your response by explicitly noting "README.md updated: <1-line summary
of what changed>" or "README.md: no update needed this time" so it's clear
you checked.
```

## 2. Keep the visualizer's own Help content in sync

Use this if you're changing the visualizer (`tools/visualizer/carpet-visualizer.html`)
itself, OR changing firmware behavior that the visualizer's Help popup
documents (its config-menu map, its "inputs available to a show" getter
reference, etc). See also prompt 6 below — a separate, standing
architectural rule (not a sync-on-change prompt like this one) that governs
*how* any code you write in this file is allowed to get its values, which
matters for any change here that touches rendering or input handling, not
just the Help text.

```text
Upon each prompt answer from now on, until further notice, for each change
currently staged in git: if any staged change affects content covered by
the visualizer's own in-app "Visualizer Help" popup
(tools/visualizer/carpet-visualizer.html, inside
<div class="help-overlay" id="helpOverlay"> ... <div class="help-modal__body">),
update that HTML content to match before your response ends.

What this popup covers (and what counts as in scope): the VISUALIZER TOOL
itself — its architecture (real firmware compiled to WASM vs. simulated
external input), which controls are real FW I/O vs. a visualizer-only
shortcut vs. simulated audio, the firmware's config-menu screen/subsetting
map as the visualizer presents it, the show/variation list, and the
developer section (repo links, the getter API reference a new light show
would use, the cheat-sheet-card generator, the WASM rebuild step). It does
NOT cover general hardware/fixture documentation unrelated to the
visualizer — that's README.md (prompt 1), reachable from this same tool via
its separate "FW readme" popup/menu item. Don't duplicate README content
here; cross-reference it instead.

Formatting — match the existing markup exactly:
- Content is structured as numbered `<h3 id="help-slug">N. Title</h3>`
  top-level sections, each `id` a short kebab-case slug prefixed `help-`
  (e.g. `help-purpose`, `help-config-tree`) — doesn't need to match the
  visible title exactly, just be short, stable, and unique.
- Subsections that are worth deep-linking to get `<h4 id="help-slug">` the
  same way (e.g. `help-fw-io`, `help-dev-repo`); a purely cosmetic
  subsection can stay a plain unlinked `<h4>`.
- The `<ul class="help-toc">` block near the top of the modal mirrors the
  heading structure exactly, as a **nested** list: one `<li><a
  href="#help-slug">N. Title</a></li>` per top-level section, and where
  that section has linked `<h4>` subsections, a nested `<ul>` of `<li><a
  href="#help-slug">Subsection title</a></li>` inside that same `<li>`
  (see the existing section 3 and section 6 entries for the exact
  pattern). Any time you add, remove, rename, renumber, or reorder a
  heading (top-level or linked subsection), update this TOC — and every
  top-level section's leading number — to match, in the same edit. A
  stale TOC entry is as much a bug as a wrong technical fact.
- **Condensed, not padded** — same standard as prompt 1's README style:
  keep every fact, tag, control name, and caveat currently present, but
  cut restatement and filler. Don't let new content you add default to a
  looser style than what's already here.
- Wrap identifiers/filenames/getter names in `<code>...</code>` (this is
  raw HTML, not markdown — literal backticks will render as backticks,
  not code spans).
- Where you're describing whether something is real firmware I/O, a
  visualizer-only dev convenience, or simulated external hardware, use the
  existing tag convention: `<span class="help-tag help-tag--fw">FW</span>`,
  `help-tag--shortcut` / `SHORTCUT`, `help-tag--sim` / `SIM` — see section 3
  ("What's real FW vs. a visualizer shortcut vs. simulated audio") for the
  definition of each.
- The "Inputs available to a show" list (section 6, for developers) is a
  getter-name reference for AudioBoard.h — if any getter is renamed, added,
  or its behavior changes (e.g. hit-decay timing, AGC modes, auto-peak),
  update this list's method names/description to match exactly, since a
  stale getter name here actively misleads a future show author.

End your response by explicitly noting "Visualizer help updated: <1-line
summary>" or "Visualizer help: no update needed this time" so it's clear
you checked.
```

## 3. Regenerate the control-box cheat sheet card

Use this if you're changing anything the printable control-box sticker
(`cheat_sheet_card.txt`/`.docx`, generated by `tools/gen_cheat_sheet_card.py`)
covers: the show/variation list or order, the config menu's top-level
screens or subsettings, or any button/pot/encoder behavior.

```text
Upon each prompt answer from now on, until further notice, for each change
currently staged in git: if any staged change affects the show/variation
list or order, the config menu's top-level screens or subsettings (in
CarpetLightLogic.cpp), or the global button/pot/encoder behavior map, then
update tools/gen_cheat_sheet_card.py's CELLS content to match, then run
`python3 tools/gen_cheat_sheet_card.py` (requires `pip install python-docx`
if not already installed) to regenerate cheat_sheet_card.txt and
cheat_sheet_card.docx at the repo root, before your response ends.

Do this by re-reading src/CarpetLightLogic.cpp, src/*Show.h, and src/Nvm.h
to confirm the CURRENT show/variation list and order, the config menu's
top-level modes and each mode's subsettings (in order), and each
show/variation's pot and encoder role plus the global button map — don't
assume anything staged already tells you the full current picture, since
CELLS has to reflect the post-change reality as a whole, not just the diff.

Keep the existing abbreviation style, column grouping, and print specs
unchanged: 6x1.5in page, ~1mm margins, Arial 10pt, 6 columns (see the
RECOMMENDED_FONT_PT/RECOMMENDED_COLUMNS/PAGE_W_IN/PAGE_H_IN constants and
the CELLS list itself in that script for the exact existing conventions —
match them, don't redesign the layout). If new content doesn't fit, trim
the least-essential detail (exact ms ranges, secondary parentheticals)
before shrinking anything else. Edit CELLS in the script — never hand-edit
the generated .txt/.docx files directly, they get overwritten.

End your response by explicitly noting "Cheat sheet card regenerated:
<1-line summary of what changed>" or "Cheat sheet card: no update needed
this time" so it's clear you checked.
```

## 4. Always end with a ready-to-paste git add/commit line

Use this any time you want Claude to stop short of actually committing, but
still hand you an exact command (or commands) reflecting everything it just
changed. Written for this specific repo's shape: a single git repository
rooted at `~/Documents/TheFlyingMagicCarpet` containing two commit
categories as sibling subdirectories — `CANTroller2/` (the vehicle control
box firmware, commit prefix `[control]`) and everything belonging to the
light show system, commit prefix `[lightbox]`: `CarpetLightCode/` itself,
`CarpetLightWifiESP/` (the WiFi radio bridge firmware), and any future
ESP-based WiFi/radio bridge codebase added for the light show system --
all of these share the `[lightbox]` prefix and count as ONE side of the
split for this rule's purposes, even though they're separate directories.
A commit must never contain changes from both categories (control vs.
lightbox), but changes spanning `CarpetLightCode/` and
`CarpetLightWifiESP/` together in one `[lightbox]` commit are fine.

```text
Upon each prompt answer from now on, until further notice: end every
response (as long as any file was created, edited, staged, or otherwise
changed during it, anywhere in the ~/Documents/TheFlyingMagicCarpet repo)
with one or more ready-to-paste shell lines reflecting what changed,
following the rules below.

This repo has two commit categories living side by side in one git repo:
CANTroller2/ ("control code," commit prefix [control]) and the light show
system ("lightbox code," commit prefix [lightbox]) -- which spans MULTIPLE
directories: CarpetLightCode/ itself, CarpetLightWifiESP/ (the WiFi radio
bridge firmware), and any future ESP-based WiFi/radio bridge codebase
added for the light show system. Treat all lightbox-prefixed directories
as one side of the split -- a single [lightbox] commit may include changes
from CarpetLightCode/ and CarpetLightWifiESP/ together, that's not mixing
categories. Only mixing control-prefixed and lightbox-prefixed changes in
one commit is disallowed. Before producing any commit line, check (e.g.
via `git status --short` from the repo root) which category(ies) actually
have changes right now -- not just what this one response touched, but
everything currently unstaged/untracked in the repo, since a stray
leftover change in the other category must not get silently swept into
this response's commit
either.

- **If changes exist in only one category**: print a single block in this
  exact form:

  git add CarpetLightCode/path/to/changed1.h CarpetLightWifiESP/path/to/changed2.cpp; git commit -m "[lightbox] 1) Section: detail 2) Section: detail"

  (or the `CANTroller2/`/`[control]` equivalent — control only ever spans
  one directory today). Note the example above deliberately shows a
  `[lightbox]` commit touching BOTH `CarpetLightCode/` and
  `CarpetLightWifiESP/` paths in one `git add` — that's normal and
  expected, not a mixing violation, since both directories share the
  `[lightbox]` category. The `git add` argument list must be the **actual
  specific changed/new file paths** (from `git status`), each one
  explicitly prefixed with its own directory — never a bare `git add .`,
  and never a directory-level `git add CarpetLightCode/` either, since
  either form silently sweeps in whatever else happens to be sitting
  unstaged in that tree (a leftover WIP change unrelated to this response)
  along with what you actually meant to commit. List every file precisely,
  every time.

- **If changes exist in BOTH categories** (control and lightbox): say so
  explicitly in your response (e.g. "Both control and lightbox code
  changed — two separate commits needed"), then print TWO separate blocks
  back to back, one per category, each in the form above with that
  category's own prefix. Never merge them into one `git add`/`git commit`
  pair, even if it feels like one logical change spanning both — if the
  change genuinely spans control and lightbox, it still gets described
  from each category's own point of view in its own commit, split at the
  category boundary (not necessarily a single directory boundary, now that
  lightbox spans more than one directory).

- **Keep the whole message short.** List each genuinely distinct change as
  its own numbered item (1), 2), 3)...) inside a given -m string — one
  item per logical/behavioral change within THAT codebase, not one item
  per file. Each item is `Section: detail` — a few words naming the
  code section/subsystem/file the change is in (e.g. `AudioBoard`,
  `pixel_war`, `README`, `visualizer help`, `BrakeControl`, `runmodes`),
  a colon, then a short, terse detail clause — a phrase, not a full
  sentence, and not the padded, rationale-heavy style used in this
  repo's actual doc prose (README.md etc.) or in your own chat responses.
  Compare:
  - Too long: "1) Redesigned AudioBoard's AutoPeak from a 15-second
    tracked-loudness scale to a per-bin 50ms hit-rate debounce that only
    gates a new edge and never interrupts an already-ongoing hit"
  - Right length: "1) AudioBoard: AutoPeak now a per-bin 50ms hit-rate
    debounce, not a 15s loudness scale"
- This applies equally to newly-created (untracked) files: include them by
  their exact path too, same as modified tracked files — `git status`
  shows both.
- Do NOT actually run `git commit` yourself — only ever print these lines
  for the user to copy/paste and run themselves. Running `git status`/
  `git add` yourself first (to check what's changed/untracked) is fine;
  running the commit is not.
- If literally nothing changed in a given response (a pure question/answer
  with no file edits, in either subtree), skip this entirely rather than
  printing an empty/no-op line.
- Edge case: a change to a file outside both `CANTroller2/` and
  `CarpetLightCode/` (e.g. the repo-root `README.md`, `.github/`) doesn't
  cleanly belong to either prefix — call this out explicitly rather than
  guessing which prefix to force it under, and ask which it should ride
  along with (or whether it needs its own commit) if it's not obvious
  from context.
```

## 5. Keep this file itself in sync

Use this so `claude_dev_prompts.md` never quietly drifts out of date with
what prompts 1-4 actually need to say — e.g. if a future change adds a new
file that needs README-syncing, changes the visualizer help's markup
structure, changes the cheat-sheet generator's constants, or otherwise
changes what detail/instructions one of the 4 prompts above ought to
include. This is the one prompt of the 5 that's about *this file*, not
about the codebase's docs/content directly.

```text
Upon each prompt answer from now on, until further notice: if anything in
this response changes what one of the standing prompts in
claude_dev_prompts.md should instruct — new content that prompt should
cover, a changed file path or markup structure it references, a changed
convention (formatting, TOC structure, slug algorithm, print spec,
whatever) it should describe, a new prompt worth adding, or an existing
instruction that's now simply wrong — update claude_dev_prompts.md to
match, in the same response. Do NOT touch a prompt's content just because
underlying code changed in a way that prompt would naturally handle on its
own next run (e.g. a new show doesn't need prompt 3's own text edited,
prompt 3 already says to re-derive the show list from source each time) —
only edit when the *instructions themselves*, not just the content they'd
produce, need to change.

Keep each prompt's own internal detail level intact: these are meant to be
detailed enough that a fresh Claude Code session, with no other context,
can reproduce the exact same content, formatting, and conventions (style,
terseness, TOC structure, etc.) as this repo currently has — not just the
general idea. When updating a prompt, preserve that same level of
specificity rather than summarizing it away.

**Flag it, don't silently absorb it.** Whenever the user's own message --
not just a code change you made -- gives an instruction meant to apply
going forward (a new standing rule, a correction to how you should behave,
a scope change to something already documented here), or implies one of
these prompts now needs updating, say so explicitly in your response
before or while acting on it -- e.g. "this is a new standing instruction,
updating prompt N" or "this changes what prompt N covers, updating it
now." Do not just quietly start behaving differently, and do not just
quietly edit this file without calling out that that's what's happening.
This applies even to small-seeming instructions -- err toward flagging.
(This exact rule was itself added because of a real miss: a git-commit
line was skipped for several responses in a row despite prompt 4 already
covering it, and README/Help-content sync was skipped despite prompts 1/2
already covering it -- both should have been caught immediately rather
than only surfacing when the user asked "where's my commit line?" and
"confirm you updated the README." Losing track of an already-documented
standing instruction is exactly as much a miss as failing to document a
new one -- both get flagged the same way.)

End your response by explicitly noting "claude_dev_prompts.md updated:
<1-line summary>" or "claude_dev_prompts.md: no update needed this time"
so it's clear you checked.
```

## 6. Respect the zero-duplication mandate for the visualizer

Not a "do X after each change" action prompt like 1-5 — a standing
architectural rule to read *before* touching `tools/visualizer/carpet-visualizer.html`
or anything it depends on (`tools/wasm/bridge/web_bridge.cpp`,
`tools/wasm/build.sh`, the Due-side `RadioLink`-style classes if/when the
realtime radio interface role exists). Paste this whenever you're about to
work on the visualizer, not on a recurring "end of response" basis.

```text
This repo has a standing architectural mandate for
tools/visualizer/carpet-visualizer.html, violated once already (found,
reported, then left unfixed for the rest of that session) and not to be
violated again without explicit case-by-case approval:

**Dev Tool role**: zero duplication of firmware logic or values in
JavaScript, ever. Every light color, every button-press classification,
every config-mode navigation state, every audio-processed value must come
from the REAL firmware running inside the compiled WASM module -- injected
in (injectPotPercent/injectEncoderDelta/injectButtonDown/Up/injectAdcBins/
etc.) and read back out (real LED buffer pointers, AudioBoard getters,
status/nav getters), never computed independently in JS. If you're about
to write JS that mimics what a C++ function in src/*.h already does --
stop, that's the violation. Add a real getter/injector to
tools/wasm/bridge/web_bridge.cpp and call that instead of reimplementing
the logic.

**Interface role** (the realtime WiFi-to-real-Due monitor/control mode, if
present): the same mandate applies, MINUS explicitly pre-approved
exceptions. As of this prompt's most recent update, there are exactly TWO:

1. **Light-value rendering.** This role deliberately renders shows locally
   in JS (`renderInterfaceMode()` and the functions it calls) from relayed
   real audio/settings data, rather than streaming real LED bytes over the
   radio link, for bandwidth reasons (see README.md / the plan file for
   the full rationale). Must be re-ported from the real FW whenever the
   corresponding `src/*Show.h` logic changes -- it does not update itself.

2. **Pot/encoder/button become monitor-only (no remote control), not a
   duplication exception exactly, but a deliberate scope restriction worth
   documenting just as explicitly.** In Interface role, the visualizer's
   pot/encoder/button panel must NEVER inject simulated input to control
   the real Due -- only Dev Tool role's local WASM path does that (see
   `web_injectPotPercent`/`web_injectEncoderDelta`/`web_injectButtonDown`/
   `Up` in `tools/wasm/bridge/web_bridge.cpp`, Dev Tool role only). Instead
   this panel is a pure one-way mirror: the pot angle line, encoder
   button, CW/CCW arrows, and press-tier labels (Short/Medium/Long/
   X-long/Double) render desaturated red by default and turn orange ONLY
   when a real event/value relayed from the Due says so (a real button-
   down/up, a real encoder step, a real live pot percentage) -- never from
   a local heuristic inferring that something is probably happening (e.g.
   watching a value drift and guessing someone's turning it). That
   distinction is exactly what keeps this compliant with the mandate
   rather than being a duplication itself -- if you ever find yourself
   writing code that infers UI activity instead of being told it
   explicitly by a relayed message, stop, that's the violation. Rationale:
   a person may be physically operating the real light box's own pot/
   encoder/button at the same time someone's watching the tablet --
   letting the tablet also drive those same inputs would mean two control
   sources fighting over one physical input. "Convenience controls" (see
   below) are NOT covered by this restriction -- those remain real
   one-way remote commands, same as ever, since setting an exact value
   doesn't have the same physical-conflict problem simulating a
   relative/physical input does.

**Term: "convenience controls".** Any visualizer UI element that reads or
writes a real FW-STORED value -- the show/variation selector, and any
slider/button/toggle corresponding to a real, persisted (or live) FW
setting (brightness levels, AGC mode, peak threshold, noise floor, hit
decay/prediction/foresight, AutoPeak, sound-reactivity, blacklight, the
AudioSource FW setting if it exists, etc.). This is the same category the
Help modal's own FW/SHORTCUT/SIM tagging already documents for Dev Tool
role (a UI shortcut that skips real hardware's multi-press menu
navigation, but the value it sets is 100% real committed firmware state)
-- convenience controls stay real, direct, one-way remote commands in
BOTH roles, not just Dev Tool. This does NOT need mandate-exception
treatment: the command goes out, the real Due's real code decides what
happens, no FW logic is duplicated locally either way. Convenience
controls are explicitly NOT the same thing as the pot/encoder/button
panel above -- that panel simulates a raw physical input (a relative
motion or a hold duration), which is exactly the physical-conflict case
this restriction exists for; a convenience control sets an exact value
instead, which doesn't have that problem. The one thing explicitly NOT a
"convenience control": anything corresponding only to a VISUALIZER-local
value with no real FW equivalent at all (the Mic/PC/File/Beats audio
source picker, the Tone generator's pattern choice, injected-noise/
source-level/phone-volume sliders, input-cap toggle -- the SIM-tagged
audio-signal-chain modeling controls) -- those are Dev-Tool-only concepts
by nature (there's no local signal chain to configure once real audio is
either coming from the real chip or relayed over radio) and simply don't
apply/appear in Interface role at all.

Everything else not covered by the two exceptions above -- button-press
classification, config-mode navigation, audio-processed values -- must
still originate from the real Due, relayed over the radio link, never
computed locally. Do NOT introduce a third exception on your own judgment,
however reasonable it seems in the moment -- if a task seems to require
one, stop and ask for explicit approval before writing the code, stating
exactly what would be duplicated (or what new scope restriction you're
proposing) and why you think it's justified. Both exceptions above must
remain reachable ONLY from Interface role -- Dev Tool role must have zero
code paths into either one; if you touch code near this boundary, verify
that's still true rather than assuming it.

If you ever find an existing violation of this mandate while working on
something unrelated, report it immediately in that same response rather
than quietly noting it and moving on -- silently deprioritizing a found
violation is the exact failure mode that already happened once and is why
this prompt exists.
```

---

## Notes for whoever's maintaining these

- Prompts 1-4 were written together in one session and cross-reference
  each other's territory (README vs. visualizer-help vs. cheat-sheet each
  have a defined, non-overlapping scope — see each prompt's own "what
  counts as in scope" paragraph) specifically so a dev can adopt any subset
  without gaps or duplicate work.
- If the visualizer's help-modal markup structure changes shape (e.g. the
  TOC/section-numbering convention changes), prompt 2 needs a matching
  update. Same for prompt 3 if `tools/gen_cheat_sheet_card.py`'s constants
  are renamed/restructured, or prompt 1 if README.md's own house style
  changes.
- Prompt 4's `[lightbox]`/`[control]` commit-tag prefixes, and the
  `CarpetLightCode`/`CANTroller2` directory split they're keyed off of,
  match this repo's existing git history convention (see `git log`) and
  actual directory layout — if either ever changes (a rename, a 3rd
  codebase added to the repo, prefixes changed), update prompt 4 to match.
  `CANTroller2/claude_dev_prompts.md` has its own copy of the commit-line
  prompt (its half of this same split-commit rule, `[control]`-prefixed
  only) — keep the shared rule (never mix codebases in one commit)
  consistent between both files if it changes.
- Prompt 5 is what's supposed to keep the 3 notes above from ever going
  stale by hand again — if a session with prompt 5 active is the one
  making a change described in those notes, it should already have updated
  the relevant prompt (and, ideally, this note) itself. These notes exist
  as a human-readable summary/backstop, not as the source of truth for
  what's in sync — the prompts' own text is.
- README.md's and the visualizer help's current table-of-contents/heading-
  anchor scheme (plain-text headings, GitHub-slug-compatible anchors, a
  `## Contents` markdown list mirrored by a nested `<ul class="help-toc">`,
  and the matching `slugify()` function in carpet-visualizer.html's
  `renderMarkdown()`) and their condensed prose style were both introduced
  in the same session that added prompt 5 — see prompts 1 and 2 above for
  the full spec of each.
