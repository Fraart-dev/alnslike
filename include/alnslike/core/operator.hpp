#pragma once
#include <alnslike/core/solution.hpp>
#include <alnslike/core/types.hpp>
#include <string_view>

namespace alnslike::core {

class LocalOperator {
public:
    virtual ~LocalOperator() = default;

    [[nodiscard]] virtual std::string_view Name() const noexcept = 0;

    // Возвращает дельту изменения целевой функции
    // Детали мутации между фазами хранит во внутреннем состоянии
    // - Если дельта считается дёшево, solution не изменяется, а finalize
    // используется для применения изменений (на тестах кратно быстрее)
    // - Если для оценки требуется изменить решение, оно выполняется здесь, а
    // finalize используется для отката
    virtual SolutionDelta Propose(Solution& solution, Rng& rng) noexcept = 0;

    // Оператор завершает ход. Применяет или откатывает изменения
    virtual void Finalize(Solution& solution, bool is_accepted) noexcept = 0;
};

class RuinOperator {
public:
    virtual ~RuinOperator() = default;

    [[nodiscard]] virtual std::string_view Name() const noexcept = 0;

    virtual SolutionDelta Ruin(Solution& solution, MoveCache& cache, Rng& rng) noexcept = 0;

    virtual void Undo(Solution& solution, MoveCache& cache) noexcept = 0;
};

class RepairOperator {
public:
    virtual ~RepairOperator() = default;

    [[nodiscard]] virtual std::string_view Name() const noexcept = 0;

    virtual SolutionDelta Repair(Solution& solution, MoveCache& cache, Rng& rng) noexcept = 0;

    virtual void Undo(Solution& solution, MoveCache& cache) noexcept = 0;
};

}  // namespace alnslike::core