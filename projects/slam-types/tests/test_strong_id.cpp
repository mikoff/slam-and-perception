/// @file test_strong_id.cpp
/// @brief GTest unit tests for slam::core::StrongId.

#include <slam/core/strong_id.hpp>

#include <gtest/gtest.h>

#include <map>
#include <sstream>
#include <unordered_map>

// --- Phantom tags for testing ---
struct AlphaTag {};
struct BetaTag {};

using AlphaId = slam::core::StrongId<AlphaTag>;
using BetaId  = slam::core::StrongId<BetaTag>;
using SmallId = slam::core::StrongId<AlphaTag, uint32_t>;

// ---- Construction & Validity -------------------------------------------

TEST(StrongId, DefaultConstructsToInvalid) {
    AlphaId id;
    EXPECT_FALSE(id.isValid());
}

TEST(StrongId, ExplicitConstructionIsValid) {
    AlphaId id(42);
    EXPECT_TRUE(id.isValid());
    EXPECT_EQ(id.value(), 42u);
}

TEST(StrongId, InvalidFactoryMatchesDefault) {
    AlphaId a;
    AlphaId b = AlphaId::Invalid();
    EXPECT_EQ(a, b);
    EXPECT_FALSE(b.isValid());
}

// ---- Copy & Move -------------------------------------------------------

TEST(StrongId, CopyPreservesValue) {
    AlphaId original(7);
    AlphaId copy = original;
    EXPECT_EQ(copy, original);
}

TEST(StrongId, MovePreservesValue) {
    AlphaId src(99);
    AlphaId dst = std::move(src);
    EXPECT_EQ(dst.value(), 99u);
}

// ---- Equality & Ordering -----------------------------------------------

TEST(StrongId, EqualIdsCompareEqual) {
    AlphaId a(10);
    AlphaId b(10);
    EXPECT_EQ(a, b);
}

TEST(StrongId, DifferentIdsCompareUnequal) {
    AlphaId a(1);
    AlphaId b(2);
    EXPECT_NE(a, b);
}

TEST(StrongId, OrderingIsConsistent) {
    AlphaId lo(1);
    AlphaId hi(5);
    EXPECT_LT(lo, hi);
    EXPECT_GT(hi, lo);
    EXPECT_LE(lo, hi);
    EXPECT_GE(hi, lo);
    EXPECT_LE(lo, lo);
    EXPECT_GE(lo, lo);
}

// ---- Type Safety (compile-time) ----------------------------------------

TEST(StrongId, DifferentTagsAreDifferentTypes) {
    // AlphaId and BetaId are distinct types at compile time.
    EXPECT_FALSE((std::is_same_v<AlphaId, BetaId>));
}

TEST(StrongId, DifferentUnderlyingTypesAreDifferentTypes) {
    EXPECT_FALSE((std::is_same_v<AlphaId, SmallId>));
}

// ---- STL Container Support ---------------------------------------------

TEST(StrongId, WorksAsUnorderedMapKey) {
    std::unordered_map<AlphaId, std::string> table;
    table[AlphaId(1)] = "one";
    table[AlphaId(2)] = "two";
    EXPECT_EQ(table.size(), 2u);
    EXPECT_EQ(table[AlphaId(1)], "one");
}

TEST(StrongId, WorksAsOrderedMapKey) {
    std::map<AlphaId, int> sorted;
    sorted[AlphaId(5)] = 50;
    sorted[AlphaId(1)] = 10;
    sorted[AlphaId(3)] = 30;
    EXPECT_EQ(sorted.begin()->first, AlphaId(1));
}

// ---- Stream Output -----------------------------------------------------

TEST(StrongId, StreamOutputShowsValue) {
    AlphaId id(123);
    std::ostringstream ss;
    ss << id;
    EXPECT_EQ(ss.str(), "123");
}

TEST(StrongId, StreamOutputShowsInvalid) {
    AlphaId id;
    std::ostringstream ss;
    ss << id;
    EXPECT_EQ(ss.str(), "INVALID");
}

// ---- Constexpr ---------------------------------------------------------

TEST(StrongId, ConstexprConstruction) {
    constexpr AlphaId id(10);
    constexpr auto val = id.value();
    static_assert(val == 10);
    EXPECT_EQ(val, 10u);
}

TEST(StrongId, ConstexprInvalid) {
    constexpr AlphaId id = AlphaId::Invalid();
    static_assert(!id.isValid());
    EXPECT_FALSE(id.isValid());
}
