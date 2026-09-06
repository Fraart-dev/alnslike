#pragma once
#include <alnslike/core/types.hpp>
#include <memory>

namespace alnslike::core {

class Solution {
public:
    virtual ~Solution() = default;

    [[nodiscard]] virtual Cost GetCost() const noexcept = 0;

    [[nodiscard]] virtual Cost GetFeasibilityViolation() const noexcept = 0;

    [[nodiscard]] virtual bool IsSolved() const noexcept { return false; }

    // Используется при инициализации для создания стартовой копии.
    [[nodiscard]] virtual std::unique_ptr<Solution> Clone() const = 0;

    // Копирует данные без выделения памяти.
    // Вызывается при обновлении лучшего решения.
    virtual void CopyFrom(const Solution& other) = 0;
};

// Общий кеш для шагов с несколькими операторами (например Ruin & Repair)
class MoveCache {
public:
    virtual ~MoveCache() = default;

    virtual void Clear() noexcept = 0;
};

}  // namespace alnslike::core