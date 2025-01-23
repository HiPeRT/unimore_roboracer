#include <gtest/gtest.h>

#include "adx_data/covariance.hpp"

namespace adx {
namespace data {

TEST(CovarianceTest, FromDoubleArray)
{
    std::array<double, 36> dArray;

    for (int i = 0; i < 36; ++i) {
        dArray[i] = static_cast<double>(i);
    }

    Covarianced<6> adx_covarianced_constructor(dArray);

    for (int i = 0; i < 36; ++i) {
        EXPECT_EQ(adx_covarianced_constructor(i % 6, i / 6), dArray[i]);
    }

    Covarianced<6> adx_covarianced_assignment = dArray;

    for (int i = 0; i < 36; ++i) {
        EXPECT_EQ(adx_covarianced_assignment(i % 6, i / 6), dArray[i]);
    }

    Covariancef<6> adx_covariancef_constructor(dArray);

    for (int i = 0; i < 36; ++i) {
        EXPECT_EQ(adx_covariancef_constructor(i % 6, i / 6), static_cast<float>(dArray[i]));
    }

    Covariancef<6> adx_covariancef_assignment = dArray;

    for (int i = 0; i < 36; ++i) {
        EXPECT_EQ(adx_covariancef_assignment(i % 6, i / 6), static_cast<float>(dArray[i]));
    }
}

TEST(CovarianceTest, ToDoubleArray)
{
    Covarianced<6> adx_covarianced;
    for (int i = 0; i < 36; ++i) {
        adx_covarianced(i % 6, i / 6) = static_cast<double>(i);
    }

    std::array<double, 36> dArray = adx_covarianced.template toArray<double>();

    for (int i = 0; i < 36; ++i) {
        EXPECT_EQ(adx_covarianced(i % 6, i / 6), dArray[i]);
    }

    Covariancef<6> adx_covariancef;
    for (int i = 0; i < 36; ++i) {
        adx_covariancef(i % 6, i / 6) = static_cast<float>(i);
    }

    dArray = adx_covariancef.template toArray<double>();

    for (int i = 0; i < 36; ++i) {
        EXPECT_EQ(adx_covariancef(i % 6, i / 6), static_cast<float>(dArray[i]));
    }
}

TEST(CovarianceTest, FromFloatArray)
{
    std::array<float, 36> fArray;

    for (int i = 0; i < 36; ++i) {
        fArray[i] = static_cast<float>(i);
    }

    Covarianced<6> adx_covarianced_constructor(fArray);

    for (int i = 0; i < 36; ++i) {
        EXPECT_EQ(static_cast<float>(adx_covarianced_constructor(i % 6, i / 6)), fArray[i]);
    }

    Covarianced<6> adx_covarianced_assignment = fArray;

    for (int i = 0; i < 36; ++i) {
        EXPECT_EQ(static_cast<float>(adx_covarianced_assignment(i % 6, i / 6)), fArray[i]);
    }

    Covariancef<6> adx_covariancef_constructor(fArray);

    for (int i = 0; i < 36; ++i) {
        EXPECT_EQ(adx_covariancef_constructor(i % 6, i / 6), fArray[i]);
    }

    Covariancef<6> adx_covariancef_assignment = fArray;

    for (int i = 0; i < 36; ++i) {
        EXPECT_EQ(adx_covariancef_assignment(i % 6, i / 6), fArray[i]);
    }
}

TEST(CovarianceTest, toFloatArray)
{
    Covarianced<6> adx_covarianced;

    for (int i = 0; i < 36; ++i) {
        adx_covarianced(i % 6, i / 6) = static_cast<double>(i);
    }

    std::array<float, 36> fArray = adx_covarianced.template toArray<float>();

    for (int i = 0; i < 36; ++i) {
        EXPECT_EQ(static_cast<float>(adx_covarianced(i % 6, i / 6)), fArray[i]);
    }

    Covariancef<6> adx_covariancef;
    for (int i = 0; i < 36; ++i) {
        adx_covariancef(i % 6, i / 6) = static_cast<float>(i);
    }

    fArray = adx_covariancef.template toArray<float>();

    for (int i = 0; i < 36; ++i) {
        EXPECT_EQ(adx_covariancef(i % 6, i / 6), fArray[i]);
    }
}

} // namespace data
} // namespace adx
