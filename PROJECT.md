# PROJECT — Arnis Create Rails

## Что это

Форк [Arnis](https://github.com/louis-e/arnis) (Rust, генерация мира Minecraft из данных OSM).
Цель форка: вместо ванильных рельсов (`railway=rail` → `minecraft:rail`) ставить рельсы мода
**Create** — блок `create:track` с корректными block entity данными соединений
(`create:track` BE: только `Connections`; `Smoothing`/`BoundLocation`/
`BoundDimension` в данных 6.0.10 отсутствуют).

Итог: сгенерированный Arnis-мир открывается в Minecraft с установленным Create,
рельсы — это треки Create, по которым ходят вагонетки Create.

## Целевые версии

| Компонент | Версия |
|---|---|
| Minecraft (Java) | 1.21.1 |
| Arnis `DATA_VERSION` | 3955 (= MC 1.21.1, `src/world_editor/java.rs:23`) |
| Create | 6.0.10+mc1.21.1 |

Риск рассинхронизации форматов минимален: версии данных совпадают.

## Текущее состояние форка

- Базовый коммит: `ab80293d` (апстрим, ветка `create-rails`).
- **Done (stages 0-1, verified in game):**
  - **Тестовый стенд** — `src/track_test_world.rs` + скрытый аргумент
    `--track-test-world=/path/to/saves/world`: детерминированный мини-мир
    128×128 (один Anvil-регион), тестовые рельсы (прямые N-S/E-W, поворот,
    уклон) + блоки `create:track` (одиночные `(10,10)` `xo`,
    `(12,12)` `zo` и прямая линия x=30, z=20..44 `shape=zo`). Не зависит от
    OSM-данных. С этапа 2 прямые сегменты стенда — это `create:track`
    (см. ниже), поворот/уклон — ванильные.
  - **Инфраструктура (блок мода в палитре)** — `block_definitions.rs`:
    `CREATE_TRACK` (ID 487) / `CREATE_FAKE_TRACK` (ID 488);
    `Block::namespace()` → `create` (для 487/488), `try_name()` →
    `track`/`fake_track`; хелпер `create_track_shape(shape)` (compound
    `{"shape": ...}`). Один ID `CREATE_TRACK` несёт все формы через свойство
    `shape` (палитра ключит записи по `(блок, свойства)`).
  - **Проверено в игре** (MC 1.21.1 + Create 6.0.10): блоки `create:track`
    (`shape=zo`/`xo`, `turn=false`, `waterlogged=false`, без BE) рендерятся
    как треки и **функциональны** — Create-вагонетка едет по прямой линии
    (x=30, z=20..44) от конца до конца. Т.е. трек с `turn=false` — настоящий
    трек, не декоративный; BE не нужен.
- **Done (stage 2): at-grade straight rails** are `create:track` (`shape=zo`/`xo`,
  рельсы теперь `create:track` (`shape=zo`/`xo`, `turn=false`, BE не
  пишется) — маппинг `at_grade_rail_props` (`railways.rs`), используется и
  пайплайном, и тестовым миром; уклоны/повороты пока ванильные
  (этапы 3–4). Поворот (48,24) в тестовом мире: как в пайплайне, ячейка
  хранит форму первого сегмента (`zo`), стык в повороте сломан до этапа 3.
- Исследование — `NOTES.md`; план работ — `ROADMAP.md`.
- `docs/create_track_samples.md` в репозитории **нет** — формат NBT описан по
  исходникам Create + blockstate JSON установленного мода (ground truth).

## Как Arnis обрабатывает рельсы (upstream-архитектура)

Точка входа — `generate_railways` (`src/element_processing/railways.rs:73`):

- `RAIL_TRACK_TYPES` — типы `railway=*`, для которых генерируется рельс
  (`rail`, `light_rail`, `subway`, `tram`, `narrow_gauge`, `monorail`, `funicular`,
  `miniature`, `preserved`, `disused`).
- Диспетчеризация:
  - `generate_rail_tunnel_shell` — рельс в туннеле;
  - `generate_rail_bridge` — рельс на мосту;
  - `generate_at_grade_rail` — рельс на поверхности (at grade).
- Выбор формы рельса:
  - `determine_rail_with_slope` (`railways.rs:694`) — уклон: если сосед выше,
    возвращается `RAIL_ASCENDING_*` в его сторону; иначе — плоская логика.
  - `determine_rail_direction` (`railways.rs:792`) — направление/поворот по
    тройке (prev, current, next): прямые (`RAIL_NORTH_SOUTH`, `RAIL_EAST_WEST`)
    и повороты (`RAIL_NORTH_EAST` и т.д.).
- `smooth_diagonal_rails` — диагональные сегменты разбиваются на осевые точки
  с промежуточными блоками.
- ID блоков (`src/block_definitions.rs:1206-1215`): `RAIL_NORTH_SOUTH` (116) …
  `RAIL_SOUTH_WEST` (125) — 10 ванильных состояний.

Ключевой вывод: **направление и уклон запечены в ID блока** (разные ID = разные
состояния). Block entity для рельсов Arnis не пишет — всё состояние закодировано
в выборе блока.

> **Изменения форка (этап 2):** `generate_at_grade_rail` пишет рельсы через
> `at_grade_rail_props`: прямые (`RAIL_NORTH_SOUTH`/`RAIL_EAST_WEST`) →
> `CREATE_TRACK` + `shape` (`zo`/`xo`); уклоны/повороты пока ванильные
> (этапы 3–4). Диспетчеризация, логика уклона, диагонали — не изменены.

## Как Arnis пишет мир

Модуль `src/world_editor/` (`mod.rs`, `common.rs`, `java.rs`, `bedrock.rs`, `luanti.rs`):

- Формат: Java Anvil (`.mca` region-файлы, палитры блоков, секция `other`),
  зависимости `fastnbt` / `fastanvil`.
- Block entities хранятся в `chunk.other["block_entities"]`.
- Вставка BE: `insert_block_entity` (`mod.rs:1358`) — **уже обобщённая**: принимает
  произвольный NBT-компound (в т.ч. `id` с любым namespace) и кладёт его в список
  `block_entities` чанка. Пример использования — `set_bed_block_entity_absolute`.
- Запись блоков: `set_block` / `set_block_with_properties` (Y-офсет от земли) /
  `set_block_absolute` / `set_block_with_properties_absolute`
  (`mod.rs:2038/2069/2093/2125`). Свойства состояния блока (state properties)
  поддерживаются: `BlockWithProperties { block, properties: Option<Arc<Value>> }`
  (`src/block_definitions.rs:59`).
- Палитра пишется как `format!("{}:{}", block.namespace(), block.name())`
  (`common.rs:779`); `Block::namespace()` зависит от ID: 487/488 → `create`,
  остальные → `minecraft` (**этап 1**, `block_definitions.rs:102`).
- `Block { id: u16 }` — блок идентифицируется только числовым ID; палитра
  сериализуется в `common.rs:777` и `java.rs:1386`.

Ключевой вывод: путь «блок + свойства состояния» существует
(`set_block_with_properties_absolute`), путь «BE произвольного типа» существует
(`insert_block_entity`), имена модов работают (`namespace()` зависит от ID —
этап 1).

## Формат Create track (Create 6.0.10+mc1.21.1)

- Блоки: `create:track` (основной) и `create:fake_track` (фейковый блок рядом
  для корректных связей/текстур).
- Block entity: `create:track` — существует **только при `turn=true`**;
  при `turn=false` BE не пишется, всё состояние — в свойствах блока.
- Свойства состояния блока:
  - `shape` — `TrackShape` (**19 значений** в 6.0.10: `zo`/`xo` прямые,
    `pd`/`nd` диагонали, `an/as/ae/aw` уклоны, `tn/ts/te/tw` телепорт,
    `cr_o`/`cr_d`/`cr_pdx`/`cr_pdz`/`cr_ndx`/`cr_ndz` пересечения, `none`;
    полный список — в `NOTES.md` §3). Блокстейт-JSON: 76 вариантов =
    19 shape × 2 `turn` × 2 `waterlogged`; модель зависит только от `shape`.
  - `turn` — bool, true = кривая с сглаживанием (наличие BE);
  - `waterlogged` — bool.
- `TrackShape` несёт оси и направления концов (`getStart`/`getEnd`, `getUp`/`getDown`)
  — по форме определяются соседние блоки и геометрия.
- Структура BE `TrackBlockEntity` (NBT) — **подтверждено дампом вайварда**:
  стандартные `id,x,y,z` (Int, абс.), `keepPacked` (Byte 0) +
  `Connections` (List<Compound>). **В данных НЕТ** `Smoothing`/
  `BoundLocation`/`BoundDimension`.
- `BezierConnection` (на ветвь, типы подтверждены): `Positions`
  (2×`{Pos: IntArray[3]}`), `Starts`/`Axes`/`Normals` (2×`{V: List<Double>[3]}`),
  `Material` (String `"create:andesite"`), `Girder` (Byte 0), `Primary`
  (Byte 0/1). Позиции/векторы **относительные** — от позиции якоря.
- **Кривая (поворот и уклон)** = 2 якорных `create:track` (`turn=true`) на
  концах + цепочка `create:fake_track` (тело дуги, минимальный BE).
  `Connections` симметричен на обоих якорях. Подробности и примеры —
  `NOTES.md` §3.
- Инвариант `validateConnections`: при загрузке/смене формы Create проверяет
  согласованность `Connections` с формой и соседями; при нарушении —
  пересчёт/очистка.
- `getConnected` (`TrackBlock.java:337-375`): для `turn=false` связи определяются
  только по осям формы (BE не читается); для `turn=true` к связям по осям формы
  добавляются связи из `Connections`.

## Данные, которые нужно считать для каждого сегмента

Для блока трека в `(x, y, z)` с соседями `prev`/`next` (точки и высоты центра линии):

| Поле | Как считается | Зависит от |
|---|---|---|
| `shape` | направление (prev→current→next) + уклон (высоты prev/current/next) | только геометрии |
| `turn` | `false` для straight/junction/crossing и «жёстких» кривых; `true` только для сглаженных кривых | решения генератора |
| `waterlogged` | `false` (или по данным воды) | окружения |
| BE `Connections` | по одному `BezierConnection` на ветвь: `Positions` (2×`{Pos:IntArray[3]}`), `Starts`/`Axes`/`Normals` (2×`{V:List<Double>[3]}`), `Material` (`"create:andesite"`), `Girder` (0), `Primary` (0/1); позиции/векторы относительные, симметрия на обоих якорях | якоря + соседей |

Критическое упрощение: **прямые (`turn=false`) — без BE**; кривые (поворот/уклон)
— 2 якоря `turn=true` (BE `Connections`) + `fake_track` (минимальный BE).
`Smoothing`/`BoundLocation`/`BoundDimension` в данных 6.0.10 отсутствуют.

## Что нужно изменить в Arnis (diff от upstream)

1. ~~**Имя блока мода**~~ — **сделано (этап 1)**: `Block::namespace()` →
   `create` для ID 487/488, `try_name()` → `track`/`fake_track`; палитра
   содержит `create:track` / `create:fake_track`.
2. ~~**Константы блоков**~~ — **сделано (этап 1)**: `CREATE_TRACK` (487),
   `CREATE_FAKE_TRACK` (488) в `src/block_definitions.rs`.
3. **Генерация** — DONE. Every rail state goes through `RailKind` -> `CREATE_TRACK` + `shape`.
   Rail topology comes from NODE GEOMETRY (`way_nodes_cells` -> `simplify_leg_nodes`
   (Douglas-Peucker) -> `plan_create_legs`); the per-cell rasterization feeds the track BED
   only — a rasterized diagonal is a staircase of `xo`/`zo` that Create cannot connect.

4. **BE для кривых** — DONE: `set_create_track_block_entity_absolute` writes `Connections`;
   `emit_create_curve`/`create_curve_emission`/`create_curve_cells` write the mirrored anchor
   pair and the `create:fake_track` chain. Handle law in code: `h = max(3.5, 7/16 * span)`
   (Create uses `r*(4/3)*tan(theta/4)` from the CORNER radius — still to be matched).

5. **`fake_track`** — по необходимости (проверить в игре, см. риски).

Формат записи (Anvil, палитры, `block_entities`) менять не нужно.

## Риски и открытые вопросы

- ~~**Имена полей NBT для Vec3** не подтверждены~~ — **решено (дамп вайварда)**:
  `V`=List<Double>[3], `Pos`=IntArray[3] (compound с полем `V`/`Pos`).
- ~~**Исходники catnip недоступны** — часть полей NBT выведена косвенно~~ —
  структура `BezierConnection` и типы подтверждены дампом вайварда
  (`NOTES.md` §3).
- ~~**`fake_track`** — семантика размещения не подтверждена~~ — **решено**:
  тело кривой (промежуточные точки дуги), минимальный BE `{id,x,y,z,keepPacked}`,
  ставится между двумя якорями `turn=true`.
- **`validateConnections`** — если `Connections` не согласованы с формой,
  Create пересчитает/очистит данные; нужно решить, писать идеальные данные
  или полагаться на пересчёт (проверить в игре).
- ~~**Namespace** — жёсткий `"minecraft"`~~ — **решено (этап 1)**:
  `Block::namespace()`/`try_name()` зависят от ID (487/488 → `create`).
- Версии данных совпадают (3955 = MC 1.21.1 = Create 6.0.10+mc1.21.1) —
  рассинхронизация маловероятна.

## Итог implementation (what exists in the fork today)

- `src/block_definitions.rs`: `CREATE_TRACK`(487)/`CREATE_FAKE_TRACK`(488),
  `create_track_shape(shape)`, `create_track_props(shape, turn)` (`turn` written only when true).
- `src/world_editor/mod.rs`: `set_create_track_block_entity_absolute`, `emit_create_curve`,
  `create_curve_emission`, `create_curve_cells`, `curve_start`. Chain rules: unit-arc-length
  sampling; cell = `(floor x, ceil(y-1e-9), floor z)`; dedupe CONSECUTIVE duplicates only
  (a global `out.contains` deletes real cells of a Bezier overshoot); anchor cells skipped by
  the caller; a chain cell never overwrites a cell that holds `create:track` (the `create:fake_track`
  block + `Some(&[])` entity clear would delete the other curve's `Connections`).
- `src/element_processing/railways.rs`: `RailKind` replaces all vanilla rail IDs;
  `way_nodes_cells` -> `simplify_leg_nodes` -> `plan_create_legs` -> `emit_create_curve`;
  straights/slopes keep upstream direction + slope logic (`determine_rail_with_slope`),
  ascending shapes `an/as/ae/aw`, and stay `turn=false` (no BE).
  Constants: `CREATE_TURN_BACKOFF=8`, `CREATE_LEG_TOLERANCE=1.5`, `CREATE_SNAP_TOL=0.09`
  (~5 deg axis/45° snap window), `CREATE_MAX_SPAN=14`, `CREATE_MAX_RUN_LEGS=3`.
- `src/track_test_world.rs` (`--track-test-world`): deterministic reference curves
  (turns r=8/12, slopes 1-in-3/5/8, both axes) — the axis-aligned system that reproduces the
  hand-built reference chains; `plan_create_legs` generalizes it to arbitrary OSM geometry.
- Verification: `cargo build` clean; `cargo test railways::tests` 14/0; full `cargo test`
  fails only `map_transformation::translate::translator::tests::test_translate_by_vector`
  (needs internet). Real-OSM worlds are validated by `tools/check_curves_be.py`; IN GAME they
  are still NOT verified — `in-game-harness` = FAILED until the user drives a generated world.
