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
    GIT_REPOSITORY https://github.com/Fraart-dev/alnslike.git
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

Поиск минимума функции `x²` для целочисленного решения. Используются:

- реализация `IntSolution` — решение, хранящее целое число `x`, целевая функция `x²`;
- реализация двух операторов `ShiftOperator`, изменяющих значение на `+1` или `-1`;
- подключение готового селектора `ClassicRouletteSelector` на основе адаптивной рулетки;
- подключение готового критерия принятия `SimulatedAnnealingCriterion`;
- сборка и запуск солвера через `SolverBuilder`.

```cpp
#include <alnslike/core/solver.hpp>
#include <alnslike/core/local_step.hpp>
#include <alnslike/builtin/criteria/simulated_annealing_criterion.hpp>
#include <alnslike/builtin/selectors/classic_roulette_selector.hpp>

#include <chrono>
#include <iostream>
#include <memory>
#include <string_view>
#include <vector>

namespace core = alnslike::core;
namespace builtin = alnslike::builtin;

class IntSolution final : public core::Solution {
public:
    explicit IntSolution(int value) : value_(value) {}
    core::Cost GetCost() const noexcept override { return value_ * value_; }
    core::Cost GetFeasibilityViolation() const noexcept override { return 0.0; }
    std::unique_ptr<Solution> Clone() const override { return std::make_unique<IntSolution>(value_); }
    bool IsSolved() const noexcept override { return value_ == 0; }
    void CopyFrom(const Solution& other) override { value_ = static_cast<const IntSolution&>(other).value_; }
    int Value() const noexcept { return value_; }
    void Shift(int delta) noexcept { value_ += delta; }
private:
    int value_;
};

class ShiftOperator final : public core::LocalOperator {
public:
    ShiftOperator(int delta, const char* name) : delta_(delta), name_(name) {}
    std::string_view Name() const noexcept override { return name_; }
    
    core::SolutionDelta Propose(core::Solution& solution, core::Rng&) noexcept override {
        const auto& sol = static_cast<const IntSolution&>(solution);
        int current = sol.Value();
        int proposed = current + delta_;
        return {.objective_delta = static_cast<double>((proposed * proposed) - (current * current)), .feasibility_delta = 0.0};
    }
    
    void Finalize(core::Solution& solution, bool is_accepted) noexcept override {
        if (is_accepted) static_cast<IntSolution&>(solution).Shift(delta_);
    }
private:
    int delta_;
    std::string name_;
};

struct DummyMoveCache final : core::MoveCache { void Clear() noexcept override {} };

int main() {
    constexpr int kInitialValue = 1000;
    constexpr std::size_t kMaxIterations = 10000;
    constexpr auto kTimeout = std::chrono::seconds{1};
    constexpr uint64_t kSeed = 33;

    std::vector<std::unique_ptr<core::LocalOperator>> operators;
    operators.push_back(std::make_unique<ShiftOperator>(1, "Increment operator"));
    operators.push_back(std::make_unique<ShiftOperator>(-1, "Decrement operator"));

    auto selector = std::make_unique<builtin::selectors::ClassicRouletteSelector>(operators.size());
    auto step = std::make_unique<core::LocalSearchStep>(std::move(selector), std::move(operators));

    using SACriterion = builtin::criteria::SimulatedAnnealingCriterion;
    auto criterion = std::make_unique<SACriterion>(
        SACriterion::Configuration{.initial_temperature = 1000.0, .cooling_rate = 0.999});

    auto solver = core::SolverBuilder{}
        .SetMaxIterations(kMaxIterations).SetTimeout(kTimeout).SetSeed(kSeed)
        .SetInitialSolution(std::make_unique<IntSolution>(kInitialValue))
        .SetSearchStep(std::move(step))
        .SetAcceptanceCriterion(std::move(criterion))
        .SetMoveCache(std::make_unique<DummyMoveCache>()).Build();

    solver->Run();
    std::cout << "Best value: " << static_cast<const IntSolution&>(solver->BestSolution()).Value() << "\n";
}
```

## Архитектура

ООП-вариант библиотеки построен на абстрактных интерфейсах с виртуальными методами. Пользователь реализует свои классы, наследующие эти интерфейсы, и передаёт их в `SolverBuilder`.

### 1. Предметная область (реализует пользователь)

- **`Solution`** — состояние текущего решения. Обязательные методы: `GetCost()`, `GetFeasibilityViolation()`, `Clone()`, `CopyFrom()`. Опциональный `IsSolved()` позволяет остановить поиск досрочно.

- **Операторы:**
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

Большинство селекторов также используют структуру конфигурации, содержащую `RewardWeights` для настройки числовых наград: `rejected` (0), `accepted` (1), `improved_objective` (2), `improved_feasibility` (3), `new_best` (6).

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
- [ ] Покрытие тестами
- [ ] Перейти от ООП-версии к концептам для устранения накладных расходов.

## Лицензия

MIT
