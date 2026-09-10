# ALNSlike

Header-only фреймворк на C++20 для проектирования и использования алгоритмов локального поиска и метаэвристик на основе Adaptive Large Neighborhood Search (ALNS). Предоставляет модульный каркас солвера, набор критериев принятия, селекторы с адаптивным выбором  операторов на основе многоруких бандитов, интерфейсы для реализации  предметных операторов.

## Возможности

- Header-only, C++20, без внешних зависимостей.
- Солвер собирается из независимых компонентов: шаг поиска, критерий принятия, селекторы операторов, операторы, генератор начального решения, постобработка. Любой компонент можно заменить собственной реализацией через интерфейс.
- Шаги поиска:
  - `AlnsStep` — последовательность ruin → repair с откатом при отклонении.
  - `LocalSearchStep` — локальный оператор с двухфазным применением (вычисление дельты и применение).
- Критерии принятия: `SimulatedAnnealingCriterion` (стандартный и с возможностью разогрева), `GreatDelugeCriterion`, `LateAcceptanceCriterion`, `StepCountingCriterion`, `DiversifiedLateAcceptanceCriterion`, `GreedyAcceptanceCriterion`, `EpsilonGreedyCriterion`.
- Селекторы операторов: `UniformRandomSelector`, `ClassicRouletteSelector`, `EpsilonGreedySelector`, `UcbSelector`, `DiscountedUcbSelector`, `ThompsonSamplingSelector`, `Exp3Selector`.
- Реализация решения и операторов для задачи коммивояжёра (TSP).
- Интерфейсы для реализации собственных `Solution`, операторов, критериев, селекторов, генераторов и постобработки.

## Быстрый старт

### Требования

Компилятор с поддержкой C++20
Для сборки тестов и примеров потребуется CMake 3.14+.

### Подключение

**Через CMake:**

```cmake
add_subdirectory(alnslike)
target_link_libraries(my_target PRIVATE alnslike::alnslike)
```

```cmake
include(FetchContent)
FetchContent_Declare(
    alnslike
    GIT_REPOSITORY [https://github.com/Fraart-dev/alnslike.git](https://github.com/Fraart-dev/alnslike.git)
    GIT_TAG main
)
FetchContent_MakeAvailable(alnslike)

target_link_libraries(your_target PRIVATE alnslike::alnslike)
```

Или напрямую через компилятор:

```bash
g++ -std=c++20 -I/path/to/alnslike/include main.cpp -o main
```

### Минимальный пример

Полный код примера находится в [`examples/minimal/main.cpp`](examples/minimal/main.cpp).

Пример демонстрирует поиск минимума функции `x²` для целочисленного решения. Используются:

- реализация `IntSolution` — решение, хранящее целое число `x`, целевая функция `x²`;
- реализация двух операторов `ShiftOperator`, изменяющих значение на `+1` или `-1`;
- подключение готового селектора `ClassicRouletteSelector` на основе адаптивной рулетки;
- подключение готового критерия принятия `SimulatedAnnealingCriterion`;
- сборка и запуск солвера через `SolverBuilder`.

## Архитектура

ООП-вариант библиотеки построен на абстрактных интерфейсах с виртуальными методами. Пользователь реализует свои классы, наследующие эти интерфейсы, и передаёт их в `SolverBuilder`.

### 1. Предметная область (реализует пользователь)

- **`Solution`** — состояние текущего решения. Обязательные методы: `GetCost()`, `GetFeasibilityViolation()`, `Clone()`, `CopyFrom()`. Опциональный `IsSolved()` позволяет остановить поиск досрочно.

- **Операторы изменения:**
  - **`LocalOperator`** — оператор локального изменения решения: `Propose()` вычисляет дельту, `Finalize()` применяет или откатывает изменение в зависимости от флага `is_accepted`.
  - **`RuinOperator`** / **`RepairOperator`** — операторы разрушения и последующего восстановления. Метод `Undo()` откатывает изменения. Сохраняют общие данные в `MoveCache`.
- **`SolutionGenerator`** (опционально) — генерирует начальное решение, если оно не задано явно.
- **`PostProcessor`** (опционально) — постобработка лучшего решения после остановки цикла.

### 2. Механизмы фреймворка (можно переопределить)

- **`SearchStep`** — абстракция одного шага поиска. Реализация выбирает оператор, применяет к текущему решению, взаимодействует с критерием принятия и возвращает `StepResult`.
  Готовые реализации:
  - *`AlnsStep`* — последовательность ruin -> repair;
  - *`LocalSearchStep`* — использует локальные операторы: `Propose()` -> критерий принятия -> `Finalize()`.
- **`OperatorSelector`** — выбирает оператор (`Select()`) и корректирует веса на основе наград (`UpdateWeights()`).
- **`AcceptanceCriterion`** — критерий принятия; `Accept()` решает, принять ли новое состояние .
- **`MoveCache`** — кеш для обмена данными между операторами внутри одного шага. Очищается перед каждым шагом.

### 3. Сборка и запуск

Солвер конструируется через `SolverBuilder`. Обязательные компоненты:

1. `SearchStep` (Конструируется с набором операторов и селектором);
2. `AcceptanceCriterion`;
3. начальное решение (`SetInitialSolution`) или его генератор (`SetGenerator`).

**Условия остановки:**

- `max_iterations` — лимит шагов;
- `max_iterations_without_improvement` — лимит шагов без улучшения лучшего решения;
- `timeout` — ограничение по времени (проверяется каждые `time_check_mask` итераций, по умолчанию 1024).

## Встроенные критерии принятия

Все классы находятся в `alnslike::builtin::criteria` и реализуют `core::AcceptanceCriterion`. Каждый критерий имеет конфигурационную структуру `Configuration` с параметрами и значениями по умолчанию.

| Критерий | Описание | Основные параметры (`Configuration`) |
| :--- | :--- | :--- |
| **`SimulatedAnnealingCriterion`** | Классическая имитация отжига. | `initial_temperature` (1000), `cooling_rate` ИЛИ `max_iterations` + `final_temperature` |
| **`HeatingSimulatedAnnealingCriterion`** | Имитация отжига с циклическим нагревом при достижении минимальной температуры. | `initial_temperature`, `minimum_temperature`, `cycle_decay_factor`, `cooling_rate` ИЛИ `iterations_per_cycle` |
| **`GreatDelugeCriterion`** | Принятие решений, не превышающих плавно снижающийся «уровень воды». | `initial_water_level` (1000), `rain_speed` (1) |
| **`LateAcceptanceCriterion`** | Принятие при сравнении со стоимостью решения N шагов назад. | `history_length` (1000) |
| **`DiversifiedLateAcceptanceCriterion`** | Вариант LAHC с обновлением истории только при интенсификации и диверсификации. | `history_length` (1000) |
| **`StepCountingCriterion`** | Сравнение с порогом, который фиксируется каждые N шагов. | `step_period` (50) |
| **`GreedyAcceptanceCriterion`** | Принимает только улучшения или равные стоимости. | *Нет параметров* |
| **`EpsilonGreedyAcceptanceCriterion`** | Улучшения всегда, ухудшения — с заданной вероятностью epsilon. | `epsilon` (0.1) |

## Встроенные селекторы операторов

Все классы находятся в `alnslike::builtin::selectors` и реализуют `core::OperatorSelector`. Конструкторы принимают количество операторов.

Большинство селекторов также используют структуру конфигурации, содержащую `RewardWeights` для настройки числовых наград: `rejected` (0), `accepted` (1), `improved_objective` (2), `improved_feasibility` (3), `new_best` (6).*

| Селектор | Описание | Основные параметры |
| :--- | :--- | :--- |
| **`UniformRandomSelector`** | Равномерный случайный выбор. | *Нет параметров* |
| **`ClassicRouletteSelector`** | Адаптивная рулетка (экспоненциальное скользящее среднее). | `reaction_factor_rho` (0.01), `min_weight` (0.01), `weights` |
| **`EpsilonGreedySelector`** | С вероятностью epsilon — случайный, иначе — лучший по средней награде. | `epsilon` (0.1), `weights` |
| **`UcbSelector`** | Классический UCB1 для стационарных сред. | `exploration_constant` (sqrt(2)), `weights` |
| **`DiscountedUcbSelector`** | Дисконтированный UCB с затуханием для нестатических сред. | `exploration_constant` (sqrt(2)), `gamma` (0.9985), `weights` |
| **`Exp3Selector`** | Алгоритм Exp3 для нестационарных сред | `gamma` (0.1), `weights` |
| **`ThompsonSamplingSelector`** | Семплирование Томпсона на основе Beta-распределения. | *Нет параметров* (нач. alpha=beta=1) |

## Roadmap

- [ ] Модуль TSP
- [ ] Реализовать Ruin & Repair операторы для TSP
- [ ] Дописать тесты
- [ ] Перейти от ООП-версии к статическому полиморфизму через концепты для устранения накладных расходов.

## Лицензия

MIT