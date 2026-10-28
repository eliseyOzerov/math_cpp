// Tests of Curves/Profile.handles (waves and keys) and Curves/Falloff.handles. Golden values are transcribed from
// graphics_cpp tests/golden/profile.json and geometry-free selection falloff formulas.
#include <cmath>
#include <vector>

#include "../Curves/Falloff.h"
#include "../Curves/Profile.h"
#include "Testing.h"

using namespace editor;
using namespace editor::profile;
using editor::testing::isApproximatelyEqual;

namespace {

bool isClose(Vector2 first, Vector2 second, float tolerance = 1e-4f) {
    return isApproximatelyEqual(first.x, second.x, tolerance) && isApproximatelyEqual(first.y, second.y, tolerance);
}

Key<float> plain(float time, float value) { return Key<float>{time, value}; }

} // namespace

// MARK: Waves

TEST(profileConstantIsTheSameEverywhere) {
    Wave three = constant(3.5f);
    for (float x : {-0.5f, 0.0f, 0.3f, 1.0f, 1.5f}) EXPECT(isApproximatelyEqual(three(x), 3.5f));
}

TEST(profileGradientRunsStraightBetweenValuesAndHoldsTheEnds) {
    // golden: gradient [0, 1] is x clamped; gradient [1, 3, 2] gives 1, 1, 1.4, 2, 3, 2.8, 2.5, 2.2, 2, 2.
    Wave ramp = gradient({0, 1});
    EXPECT(isApproximatelyEqual(ramp(0.1f), 0.1f));
    EXPECT(isApproximatelyEqual(ramp(-0.5f), 0));
    EXPECT(isApproximatelyEqual(ramp(1.5f), 1));
    Wave bumpy = gradient({1, 3, 2});
    EXPECT(isApproximatelyEqual(bumpy(-0.5f), 1));
    EXPECT(isApproximatelyEqual(bumpy(0.1f), 1.4f));
    EXPECT(isApproximatelyEqual(bumpy(0.25f), 2));
    EXPECT(isApproximatelyEqual(bumpy(0.5f), 3));
    EXPECT(isApproximatelyEqual(bumpy(0.6f), 2.8f, 1e-5f));
    EXPECT(isApproximatelyEqual(bumpy(0.75f), 2.5f));
    EXPECT(isApproximatelyEqual(bumpy(1), 2));
    EXPECT(isApproximatelyEqual(gradient({2})(0.7f), 2));
    EXPECT(std::isnan(ramp(std::nanf(""))));
    EXPECT_PROBLEM(gradient({}), "needs a value");
}

TEST(profileStepsJumpBetweenEqualParts) {
    // golden: steps [1, 2, 3] -> 1 below a third, 2 in the middle third, 3 in the last, and 3 at 1 and beyond.
    Wave bands = steps({1, 2, 3});
    EXPECT(isApproximatelyEqual(bands(-0.5f), 1));
    EXPECT(isApproximatelyEqual(bands(0), 1));
    EXPECT(isApproximatelyEqual(bands(0.25f), 1));
    EXPECT(isApproximatelyEqual(bands(0.5f), 2));
    EXPECT(isApproximatelyEqual(bands(0.75f), 3));
    EXPECT(isApproximatelyEqual(bands(1), 3));
    EXPECT(isApproximatelyEqual(bands(1.5f), 3));
    EXPECT(isApproximatelyEqual(steps({4})(0.5f), 4));
    EXPECT_PROBLEM(steps({}), "need a value");
}

TEST(profileSineAndCosineMatchTheGoldenValues) {
    // golden: sine at 0, 0.1, 0.25, 0.6, 0.75 = 0, 0.5877853, 1, -0.5877853, -1; cosine at 0, 0.1, 0.5 = 1, 0.809017, -1.
    Wave sineWave = sine();
    EXPECT(isApproximatelyEqual(sineWave(0), 0, 1e-6f));
    EXPECT(isApproximatelyEqual(sineWave(0.1f), 0.5877853f, 1e-5f));
    EXPECT(isApproximatelyEqual(sineWave(0.25f), 1, 1e-5f));
    EXPECT(isApproximatelyEqual(sineWave(0.6f), -0.5877853f, 1e-5f));
    EXPECT(isApproximatelyEqual(sineWave(0.75f), -1, 1e-5f));
    Wave cosineWave = cosine();
    EXPECT(isApproximatelyEqual(cosineWave(0), 1, 1e-6f));
    EXPECT(isApproximatelyEqual(cosineWave(0.1f), 0.809017f, 1e-5f));
    EXPECT(isApproximatelyEqual(cosineWave(0.5f), -1, 1e-5f));
    // golden: three cycles of sine at 0.1 -> 0.9510565.
    EXPECT(isApproximatelyEqual(sine(Swing{.cycles = 3})(0.1f), 0.9510565f, 1e-5f));
}

TEST(profileWavesRepeatEveryCycleAndSwingBetweenLowAndHigh) {
    // Property: a wave of 3 cycles repeats every third, and stays between low and high.
    Swing layout{.cycles = 3, .phase = 0.2f, .low = 2, .high = 10};
    for (Wave wave : {sine(layout), cosine(layout), triangle(layout), saw(layout)}) {
        for (float x = 0; x < 0.6f; x += 0.037f) {
            EXPECT(isApproximatelyEqual(wave(x), wave(x + 1.0f / 3), 1e-3f));
            EXPECT(wave(x) >= 2 - 1e-4f && wave(x) <= 10 + 1e-4f);
        }
    }
    EXPECT(isApproximatelyEqual(sine(Swing{.low = 0, .high = 10})(0.25f), 10, 1e-4f)); // hand: the top of the sine
    EXPECT(isApproximatelyEqual(sine(Swing{.low = 0, .high = 10})(0), 5, 1e-4f));      // the middle at the start
    EXPECT(isApproximatelyEqual(saw(Swing{.phase = 0.5f})(0), 0, 1e-6f));              // starting half a cycle in
}

TEST(profileSquareTriangleAndSawMatchTheGoldenValues) {
    // golden square: 1 at 0, 0.1, 0.25; -1 at 0.5, 0.6, 0.75; 1 at 1. Three cycles at 0.25: -1.
    Wave squareWave = square();
    EXPECT(isApproximatelyEqual(squareWave(0), 1));
    EXPECT(isApproximatelyEqual(squareWave(0.25f), 1));
    EXPECT(isApproximatelyEqual(squareWave(0.5f), -1));
    EXPECT(isApproximatelyEqual(squareWave(0.75f), -1));
    EXPECT(isApproximatelyEqual(squareWave(1), 1));
    EXPECT(isApproximatelyEqual(square(SquareSwing{{.cycles = 3}})(0.25f), -1));
    SquareSwing quarterDuty;
    quarterDuty.duty = 0.25f;
    EXPECT(isApproximatelyEqual(square(quarterDuty)(0.2f), 1));
    EXPECT(isApproximatelyEqual(square(quarterDuty)(0.3f), -1));
    // golden triangle: -1, -0.6, 0, 1 at 0, 0.1, 0.25, 0.5. Saw: -1, -0.8, -0.5, 0, 0.5, -1 at 0, 0.1, 0.25, 0.5, 0.75, 1.
    Wave triangleWave = triangle();
    EXPECT(isApproximatelyEqual(triangleWave(0), -1));
    EXPECT(isApproximatelyEqual(triangleWave(0.1f), -0.6f, 1e-5f));
    EXPECT(isApproximatelyEqual(triangleWave(0.25f), 0, 1e-6f));
    EXPECT(isApproximatelyEqual(triangleWave(0.5f), 1));
    Wave sawWave = saw();
    EXPECT(isApproximatelyEqual(sawWave(0), -1));
    EXPECT(isApproximatelyEqual(sawWave(0.1f), -0.8f, 1e-5f));
    EXPECT(isApproximatelyEqual(sawWave(0.25f), -0.5f));
    EXPECT(isApproximatelyEqual(sawWave(0.5f), 0));
    EXPECT(isApproximatelyEqual(sawWave(0.75f), 0.5f));
    EXPECT(isApproximatelyEqual(sawWave(1), -1));
}

TEST(profileWaveAcceptsANumberAListOrAWave) {
    // golden: waveAt 2 -> 2 everywhere; waveAt [1, 3, 2] at 0.1 -> 1.4.
    EXPECT(isApproximatelyEqual(wave(Strength(2.0f))(0.4f), 2));
    EXPECT(isApproximatelyEqual(wave(Strength(std::vector<float>{1, 3, 2}))(0.1f), 1.4f));
    EXPECT(isApproximatelyEqual(wave(Strength(Wave(saw())))(0.5f), 0));
}

TEST(profileTimesMultipliesAndPlusAdds) {
    // golden timesAt [2, [0, 1], sine x2] at 0.1 = 0.1902113; plusAt [1, [0, 1], cosine x2] at 0.1 = 1.409017.
    Wave product = times({Strength(2.0f), Strength(std::vector<float>{0, 1}), Strength(Wave(sine(Swing{.cycles = 2})))});
    EXPECT(isApproximatelyEqual(product(0.1f), 0.1902113f, 1e-5f));
    Wave sum = plus({Strength(1.0f), Strength(std::vector<float>{0, 1}), Strength(Wave(cosine(Swing{.cycles = 2})))});
    EXPECT(isApproximatelyEqual(sum(0.1f), 1.409017f, 1e-5f));
    // Neutral elements: no factors give 1, no terms give 0.
    EXPECT(isApproximatelyEqual(times({})(0.3f), 1));
    EXPECT(isApproximatelyEqual(plus({})(0.3f), 0));
    // Property: a wave times the constant 1 is itself; plus the constant 0 too.
    Wave base = triangle(Swing{.cycles = 2});
    EXPECT(isApproximatelyEqual(times({Strength(Wave(base)), Strength(1.0f)})(0.37f), base(0.37f)));
    EXPECT(isApproximatelyEqual(plus({Strength(Wave(base)), Strength(0.0f)})(0.37f), base(0.37f)));
}

// MARK: Keys

TEST(profileKeysAtWithNoKeysIsTheFallback) {
    EXPECT(isApproximatelyEqual(keysAt<float>(Keys<float>{}, 0.5f), 0));
    EXPECT(isApproximatelyEqual(keysAt<float>(Keys<float>{}, 0.5f, 7), 7));
    EXPECT(isApproximatelyEqual(keysAt<float>(std::optional<Keys<float>>(), 0.5f, 7), 7));
}

TEST(profileKeysAtRunsStraightHoldsTheEndsAndHandlesTheCubic) {
    Keys<float> straight = {plain(0, 0), plain(1, 4)};
    EXPECT(isApproximatelyEqual(keysAt(straight, 0.25f), 1)); // hand: 4 x 0.25
    EXPECT(isApproximatelyEqual(keysAt(straight, -1.0f), 0)); // level before the first
    EXPECT(isApproximatelyEqual(keysAt(straight, 2.0f), 4));  // and past the last
    // golden: ease keys (0.2, 1) to (0.8, 4) have the value 2.5 halfway; the curve is level at both ends.
    Keys<float> eased = easeKeys(0.2f, 1, 0.8f, 4);
    EXPECT(isApproximatelyEqual(keysAt(eased, 0.5f), 2.5f, 1e-3f));
    EXPECT(isApproximatelyEqual(keysAt(eased, 0.2f), 1));
    EXPECT(isApproximatelyEqual(keysAt(eased, 0.8f), 4));
    // golden: three keys with handles at t = 0.25 give 1.2942997.
    Keys<float> handled = {Key<float>{0, 0, Handles{std::nullopt, Vector2(0.3f, 1)}},
                           Key<float>{0.5f, 2, Handles{Vector2(-0.2f, 0.5f), Vector2(0.1f, -1)}},
                           Key<float>{1, 1, Handles{Vector2(-0.4f, 0), std::nullopt}}};
    EXPECT(isApproximatelyEqual(keysAt(handled, 0.25f), 1.2942997f, 1e-3f));
    EXPECT(isApproximatelyEqual(keysAt(handled, 0.5f), 2, 1e-3f)); // a key's own value
}

TEST(profileKeysAtOfASlideKeysAtTheSameTimeTakesTheLaterValue) {
    Keys<float> jump = {plain(0, 1), plain(0.5f, 1), plain(0.5f, 5), plain(1, 5)};
    EXPECT(isApproximatelyEqual(keysAt(jump, 0.25f), 1));
    EXPECT(isApproximatelyEqual(keysAt(jump, 0.75f), 5));
}

TEST(profileKeysAtBlendsOtherValuesByMix) {
    // golden keysAtVec: (0, (0, 10)), (0.5, (4, 0)) at 0.25 -> (2, 5).
    Keys<Vector2> corners = {Key<Vector2>{0, Vector2(0, 10)}, Key<Vector2>{0.5f, Vector2(4, 0)}, Key<Vector2>{1, Vector2(2, 2)}};
    std::function<Vector2(const Vector2&, const Vector2&, float)> mix = [](const Vector2& start, const Vector2& end, float amount) {
        return Vector2::linearBlend(start, end, amount);
    };
    EXPECT(isClose(keysAt<Vector2>(corners, 0.25f, Vector2(9, 9), mix), Vector2(2, 5)));
    EXPECT(isClose(keysAt<Vector2>(corners, -1.0f, Vector2(9, 9), mix), Vector2(0, 10)));
    EXPECT(isClose(keysAt<Vector2>(Keys<Vector2>{}, 0.5f, Vector2(9, 9), mix), Vector2(9, 9)));
    EXPECT_PROBLEM(keysAt<Vector2>(corners, 0.25f), "needs a mix");
}

TEST(profileSortedKeepsTiesInOrder) {
    // golden: sorted [(1, 1), (0, 2), (0.5, 3), (0.5, 4)] = [(0, 2), (0.5, 3), (0.5, 4), (1, 1)].
    std::optional<Keys<float>> ordered = sorted<float>(Keys<float>{plain(1, 1), plain(0, 2), plain(0.5f, 3), plain(0.5f, 4)});
    EXPECT(ordered.has_value());
    EXPECT_EQUAL(ordered->size(), size_t(4));
    EXPECT(isApproximatelyEqual((*ordered)[0].value, 2));
    EXPECT(isApproximatelyEqual((*ordered)[1].value, 3));
    EXPECT(isApproximatelyEqual((*ordered)[2].value, 4));
    EXPECT(isApproximatelyEqual((*ordered)[3].value, 1));
    EXPECT(!sorted<float>(std::nullopt).has_value());
    EXPECT(!sorted<float>(Keys<float>{}).has_value());
}

TEST(profileSpreadPlacesValuesEvenly) {
    // golden: spread [5] = [(0, 5)]; spread [1, 2, 3] = [(0, 1), (0.5, 2), (1, 3)].
    Keys<float> one = spread<float>({5});
    EXPECT_EQUAL(one.size(), size_t(1));
    EXPECT(isApproximatelyEqual(one[0].time, 0));
    Keys<float> three = spread<float>({1, 2, 3});
    EXPECT(isApproximatelyEqual(three[1].time, 0.5f));
    EXPECT(isApproximatelyEqual(three[1].value, 2));
    EXPECT(isApproximatelyEqual(three[2].time, 1));
    EXPECT(spread<float>({}).empty());
}

TEST(profileEaseKeysAndBulgeMatchTheGolden) {
    // golden: easeKeys(0.2, 1, 0.8, 4) handles are 0.3 long and flat; easeKeys(0, 0, 1, 1) 0.5 long.
    Keys<float> eased = easeKeys(0.2f, 1, 0.8f, 4);
    EXPECT(isClose(*eased[0].handles->out, Vector2(0.3f, 0)));
    EXPECT(isClose(*eased[1].handles->in, Vector2(-0.3f, 0)));
    EXPECT(!eased[0].handles->in.has_value() && !eased[1].handles->out.has_value());
    // golden: bulge(0.4) handles (1/3, -0.8); bulge(2, 1) handles (1/3, 4/3); the middle reaches mid.
    Keys<float> waist = bulge(0.4f);
    EXPECT(isClose(*waist[0].handles->out, Vector2(1.0f / 3, -0.8f)));
    EXPECT(isClose(*waist[1].handles->in, Vector2(-1.0f / 3, -0.8f)));
    EXPECT(isClose(*bulge(2, 1)[0].handles->out, Vector2(1.0f / 3, 4.0f / 3)));
    EXPECT(isApproximatelyEqual(keysAt(waist, 0.5f), 0.4f, 1e-3f));
    EXPECT(isApproximatelyEqual(keysAt(bulge(2, 1), 0.5f), 2, 1e-3f));
    EXPECT(isApproximatelyEqual(keysAt(waist, 0), 1));
    EXPECT(isApproximatelyEqual(keysAt(waist, 1), 1));
    // Property: a bulge is symmetric about the middle.
    EXPECT(isApproximatelyEqual(keysAt(waist, 0.2f), keysAt(waist, 0.8f), 1e-3f));
}

TEST(profileKeysOfTakesLooseInput) {
    EXPECT(!keysOf(std::nullopt).has_value());
    // golden: keysOf 3 = [(0, 3)].
    std::optional<Keys<float>> number = keysOf(KeysInput(3.0f));
    EXPECT(number.has_value() && number->size() == 1 && isApproximatelyEqual((*number)[0].value, 3));
    std::optional<Keys<float>> list = keysOf(KeysInput(std::vector<float>{1, 2, 3}));
    EXPECT(list.has_value() && list->size() == 3 && isApproximatelyEqual((*list)[1].time, 0.5f));
    EXPECT(keysOf(KeysInput(std::vector<float>{}))->empty());
    Keys<float> given = {plain(0, 1), plain(1, 2)};
    EXPECT(keysOf(KeysInput(given))->size() == 2);
    // golden keysOfSpec: a saw wave in 5 samples = [(0, -1), (0.25, -0.5), (0.5, 0), (0.75, 0.5), (1, -1)].
    std::optional<Keys<float>> sampled = keysOf(KeysInput(Wave(saw())), 5);
    EXPECT(sampled.has_value() && sampled->size() == 5);
    EXPECT(isApproximatelyEqual((*sampled)[1].time, 0.25f));
    EXPECT(isApproximatelyEqual((*sampled)[1].value, -0.5f));
    EXPECT(isApproximatelyEqual((*sampled)[3].value, 0.5f));
    EXPECT(isApproximatelyEqual((*sampled)[4].value, -1));
    EXPECT_EQUAL(keysOf(KeysInput(Wave(sine())))->size(), size_t(65)); // the default sample count
}

TEST(profileKeysTimesMultipliesOnAFineGrid) {
    // golden: a missing side returns the other; two missing is none.
    Keys<float> ramp = spread<float>({0, 1});
    EXPECT(!keysTimes(std::nullopt, std::nullopt).has_value());
    EXPECT_EQUAL(keysTimes(ramp, std::nullopt)->size(), size_t(2));
    EXPECT_EQUAL(keysTimes(std::nullopt, ramp)->size(), size_t(2));
    // Hand: (2 everywhere) x (0 to 1) is 0 to 2, so the value at t is 2t, on a 129-point grid in order.
    std::optional<Keys<float>> doubled = keysTimes(Keys<float>{plain(0, 2), plain(1, 2)}, ramp);
    EXPECT(doubled.has_value());
    EXPECT_EQUAL(doubled->size(), size_t(129));
    for (const Key<float>& key : *doubled) EXPECT(isApproximatelyEqual(key.value, 2 * key.time, 1e-5f));
    for (size_t index = 1; index < doubled->size(); index += 1) EXPECT((*doubled)[index].time > (*doubled)[index - 1].time);
    // A key time off the grid is added, so a step stays sharp.
    EXPECT_EQUAL(keysTimes(Keys<float>{plain(0, 1), plain(0.3f, 1), plain(1, 1)}, ramp)->size(), size_t(130));
}

// MARK: Falloff

TEST(falloffWeightsAtTheCentreAndTheEdge) {
    // Hand: every shape is full at the centre; all but `constant` are zero at the edge.
    for (Falloff falloff : {Falloff::smooth, Falloff::linear, Falloff::sphere, Falloff::root, Falloff::sharp, Falloff::constant}) {
        EXPECT(isApproximatelyEqual(falloffWeight(falloff, 0), 1));
    }
    for (Falloff falloff : {Falloff::smooth, Falloff::linear, Falloff::sphere, Falloff::root, Falloff::sharp}) {
        EXPECT(isApproximatelyEqual(falloffWeight(falloff, 1), 0));
    }
    EXPECT(isApproximatelyEqual(falloffWeight(Falloff::constant, 1), 1));
}

TEST(falloffWeightsHalfwayMatchTheirFormulas) {
    // Hand at share 0.5: smooth 1 - 0.5 = 0.5; linear 0.5; sphere sqrt(0.75); root sqrt(0.5); sharp 0.25; constant 1.
    EXPECT(isApproximatelyEqual(falloffWeight(Falloff::smooth, 0.5f), 0.5f));
    EXPECT(isApproximatelyEqual(falloffWeight(Falloff::linear, 0.5f), 0.5f));
    EXPECT(isApproximatelyEqual(falloffWeight(Falloff::sphere, 0.5f), 0.8660254f, 1e-5f));
    EXPECT(isApproximatelyEqual(falloffWeight(Falloff::root, 0.5f), 0.7071068f, 1e-5f));
    EXPECT(isApproximatelyEqual(falloffWeight(Falloff::sharp, 0.5f), 0.25f));
    EXPECT(isApproximatelyEqual(falloffWeight(Falloff::constant, 0.5f), 1));
}

TEST(falloffNeverIncreasesAndHoldsItsShareInRange) {
    for (Falloff falloff : {Falloff::smooth, Falloff::linear, Falloff::sphere, Falloff::root, Falloff::sharp, Falloff::constant}) {
        float previous = 1;
        for (float share = 0; share <= 1.0001f; share += 0.05f) {
            float weight = falloffWeight(falloff, share);
            EXPECT(weight <= previous + 1e-6f);
            EXPECT(weight >= 0 && weight <= 1);
            previous = weight;
        }
        EXPECT(isApproximatelyEqual(falloffWeight(falloff, -3), falloffWeight(falloff, 0))); // held at the centre
        EXPECT(isApproximatelyEqual(falloffWeight(falloff, 4), falloffWeight(falloff, 1)));  // and at the edge
    }
}
