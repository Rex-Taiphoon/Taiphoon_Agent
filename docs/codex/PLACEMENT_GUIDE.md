# 檔案放置與 Codex 使用方式

## 建議 Repository 結構

把本 package 內容合併進 `Taiphoon_Agent`：

```text
Taiphoon_Agent/
├── AGENTS.md                       # 現有，依 Patch Guide 小幅更新
├── CODE_CONSTRAINT.md
├── CONTEXT.md
├── CHARACTERS.md
│
├── docs/
│   └── codex/
│       ├── README.md               # Codex 入口
│       ├── EGO_SWARM_ARCHITECTURE.md
│       ├── EGO_SWARM_IMPLEMENTATION_GUIDE.md
│       ├── EGO_SWARM_CODEX_PROMPTS.md
│       ├── PHASE_STATUS.md
│       ├── AGENTS_PATCH_GUIDE.md
│       └── PLACEMENT_GUIDE.md
│
├── src/
│   ├── AGENTS.md
│   │
│   ├── ego_swarm/
│   │   ├── AGENTS.md               # 新增，最重要的 scoped rule
│   │   └── ...
│   │
│   ├── perception/
│   │   ├── AGENTS.md               # 新增
│   │   └── ...
│   │
│   ├── nodes/
│   │   ├── AGENTS.md
│   │   └── ...
│   ├── controllers/
│   │   ├── AGENTS.md
│   │   └── ...
│   ├── safety/
│   ├── formation/                  # legacy
│   └── trajectory/                 # legacy
│
└── AutoTest/
    └── AGENTS.md
```

---

# 為什麼這樣擺

## `AGENTS.md`

放「一定要遵守」的短規則。

優點：

Codex 修改某個 subtree 時可直接得到 scoped instructions。

不要把完整 roadmap 全塞進 AGENTS。

否則：

- 每個任務都讀到大量不相關內容
- 更容易出現 instruction conflict
- Phase scope 不容易維護

---

## `docs/codex/EGO_SWARM_ARCHITECTURE.md`

放長期架構。

不應隨單一 Phase 大量變動。

---

## `docs/codex/EGO_SWARM_IMPLEMENTATION_GUIDE.md`

放開發順序與 acceptance criteria。

它是 roadmap。

---

## `docs/codex/EGO_SWARM_CODEX_PROMPTS.md`

放單次任務。

使用時只取一個 Phase。

---

## `PHASE_STATUS.md`

避免新的 Codex session 不知道前一 Phase：

- 有沒有完成
- 哪些測試跑過
- 哪些風險尚未接受

---

# Codex 最有效的工作方式

## Step 1 — 第一次整理 repo

先只做文件與 AGENTS alignment。

不要同時開始 Phase 0 code。

建議人工 review：

```text
docs/codex/*
src/ego_swarm/AGENTS.md
src/perception/AGENTS.md
existing AGENTS patches
```

---

## Step 2 — 開始 Phase 0

建立新的 Codex task。

給它：

```text
請依照 docs/codex/EGO_SWARM_CODEX_PROMPTS.md 的 Common Prefix
與 Phase 0 Prompt 執行。
```

不需要每次重新貼全部架構。

因為 prompt 會要求它讀 repository 文件。

---

## Step 3 — Review

Phase 完成後：

1. review diff
2. review tests
3. review runtime behavior
4. 更新 `PHASE_STATUS.md`
5. 人工將 Phase 改成 `Accepted`

---

## Step 4 — 下一 Phase

新的 Codex task：

```text
請依照 docs/codex/EGO_SWARM_CODEX_PROMPTS.md 的 Common Prefix
與 Phase 1 Prompt 執行。

Phase 0 已 Accepted，請以 PHASE_STATUS.md 為準。
```

---

# 不建議的使用方式

不要：

```text
「幫我把 Phase 0 到 Phase 6 全部做完」
```

不要：

```text
「參考 EGO-Swarm 然後重構整個 repo」
```

不要把所有 Phase prompt 一次貼給 Codex 要它自行決定。

這會造成：

- scope creep
- safety contract 與 optimizer 混在同一 diff
- 難以 review
- 難以定位 SITL regression

---

# 建議每個 Phase 使用獨立 Branch（可選）

例如：

```text
ego/phase-0-baseline
ego/phase-1-swarm-safety
ego/phase-2-obstacle-map
ego/phase-3-trajectory-state
ego/phase-4-dynamic-safety
ego/phase-5-formation
ego/phase-6-optimizer
```

如果你目前習慣全部在 `EGO-Swarm-dev` 上開發，也可以不採此策略。

重點是：

每個 Phase 必須保持可辨識、可 review、可 rollback。

---

# 最低限度 Codex 啟動提示

文件放好後，日常其實只需要：

```text
請閱讀 repository 的 AGENTS.md、docs/codex/README.md 與
docs/codex/EGO_SWARM_CODEX_PROMPTS.md。

執行 Phase N，只做此 Phase。
開始修改前先輸出 architecture/scope/test plan；
完成後依 docs/codex/README.md 的格式回報。
```
