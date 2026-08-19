#include <GLFW/glfw3.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace {

constexpr double PI = 3.14159265358979323846;
constexpr double PPI_DYNAMIC_RANGE_DB = 60.0;

struct PpiPoint {
    float x;
    float y;
    float intensity;
    float rangeM;
    std::size_t azimuthIndex;
    std::size_t binIndex;
};

struct PpiTarget {
    int id;
    float x;
    float y;
    float strength;
    float rangeM;
    float bearingDeg;
};

struct PpiUiState {
    float gain{0.82f};
    float seaClutter{0.22f};
    float rainClutter{0.08f};
    float persistence{0.78f};
    bool visualSeaTexture{false};
    bool targetOverlay{true};
};


struct Colour {
    float r;
    float g;
    float b;
    float a;
};

void drawFilledRect(
    float x,
    float y,
    float width,
    float height,
    Colour colour) {

    glColor4f(
        colour.r,
        colour.g,
        colour.b,
        colour.a);

    glBegin(GL_QUADS);
    glVertex2f(x, y);
    glVertex2f(x + width, y);
    glVertex2f(x + width, y + height);
    glVertex2f(x, y + height);
    glEnd();
}

void drawRect(
    float x,
    float y,
    float width,
    float height,
    Colour colour,
    float lineWidth = 1.0f) {

    glLineWidth(lineWidth);

    glColor4f(
        colour.r,
        colour.g,
        colour.b,
        colour.a);

    glBegin(GL_LINE_LOOP);
    glVertex2f(x, y);
    glVertex2f(x + width, y);
    glVertex2f(x + width, y + height);
    glVertex2f(x, y + height);
    glEnd();

    glLineWidth(1.0f);
}

using Glyph = std::array<std::uint8_t, 7>;

Glyph glyphFor(char input) {
    const char c = static_cast<char>(
        std::toupper(
            static_cast<unsigned char>(input)));

    switch (c) {
        case '0': return {14, 17, 19, 21, 25, 17, 14};
        case '1': return {4, 12, 4, 4, 4, 4, 14};
        case '2': return {14, 17, 1, 2, 4, 8, 31};
        case '3': return {30, 1, 1, 14, 1, 1, 30};
        case '4': return {2, 6, 10, 18, 31, 2, 2};
        case '5': return {31, 16, 16, 30, 1, 1, 30};
        case '6': return {14, 16, 16, 30, 17, 17, 14};
        case '7': return {31, 1, 2, 4, 8, 8, 8};
        case '8': return {14, 17, 17, 14, 17, 17, 14};
        case '9': return {14, 17, 17, 15, 1, 1, 14};

        case 'A': return {14, 17, 17, 31, 17, 17, 17};
        case 'B': return {30, 17, 17, 30, 17, 17, 30};
        case 'C': return {14, 17, 16, 16, 16, 17, 14};
        case 'D': return {30, 17, 17, 17, 17, 17, 30};
        case 'E': return {31, 16, 16, 30, 16, 16, 31};
        case 'F': return {31, 16, 16, 30, 16, 16, 16};
        case 'G': return {14, 17, 16, 23, 17, 17, 15};
        case 'H': return {17, 17, 17, 31, 17, 17, 17};
        case 'I': return {14, 4, 4, 4, 4, 4, 14};
        case 'J': return {7, 2, 2, 2, 2, 18, 12};
        case 'K': return {17, 18, 20, 24, 20, 18, 17};
        case 'L': return {16, 16, 16, 16, 16, 16, 31};
        case 'M': return {17, 27, 21, 21, 17, 17, 17};
        case 'N': return {17, 25, 21, 19, 17, 17, 17};
        case 'O': return {14, 17, 17, 17, 17, 17, 14};
        case 'P': return {30, 17, 17, 30, 16, 16, 16};
        case 'Q': return {14, 17, 17, 17, 21, 18, 13};
        case 'R': return {30, 17, 17, 30, 20, 18, 17};
        case 'S': return {15, 16, 16, 14, 1, 1, 30};
        case 'T': return {31, 4, 4, 4, 4, 4, 4};
        case 'U': return {17, 17, 17, 17, 17, 17, 14};
        case 'V': return {17, 17, 17, 17, 17, 10, 4};
        case 'W': return {17, 17, 17, 21, 21, 21, 10};
        case 'X': return {17, 17, 10, 4, 10, 17, 17};
        case 'Y': return {17, 17, 10, 4, 4, 4, 4};
        case 'Z': return {31, 1, 2, 4, 8, 16, 31};

        case '.': return {0, 0, 0, 0, 0, 12, 12};
        case ':': return {0, 12, 12, 0, 12, 12, 0};
        case '-': return {0, 0, 0, 31, 0, 0, 0};
        case '+': return {0, 4, 4, 31, 4, 4, 0};
        case '/': return {1, 2, 2, 4, 8, 8, 16};
        case '%': return {17, 2, 4, 8, 16, 17, 0};
        case '(': return {2, 4, 8, 8, 8, 4, 2};
        case ')': return {8, 4, 2, 2, 2, 4, 8};
        case '[': return {14, 8, 8, 8, 8, 8, 14};
        case ']': return {14, 2, 2, 2, 2, 2, 14};
        case '<': return {1, 2, 4, 8, 4, 2, 1};
        case '>': return {16, 8, 4, 2, 4, 8, 16};
        case '_': return {0, 0, 0, 0, 0, 0, 31};
        case ' ': return {0, 0, 0, 0, 0, 0, 0};
        default:  return {0, 0, 0, 0, 0, 0, 0};
    }
}

float textWidth(
    const std::string& text,
    float pixelSize) {

    return static_cast<float>(text.size()) *
        6.0f * pixelSize;
}

void drawText(
    float x,
    float y,
    float pixelSize,
    const std::string& text,
    Colour colour) {

    glColor4f(
        colour.r,
        colour.g,
        colour.b,
        colour.a);

    glBegin(GL_QUADS);

    float cursorX = x;

    for (char c : text) {
        const Glyph glyph = glyphFor(c);

        for (int row = 0; row < 7; ++row) {
            for (int column = 0; column < 5; ++column) {
                const std::uint8_t mask =
                    static_cast<std::uint8_t>(
                        1u << (4 - column));

                if ((glyph[row] & mask) == 0u) {
                    continue;
                }

                const float x0 =
                    cursorX +
                    static_cast<float>(column) *
                        pixelSize;

                const float y0 =
                    y +
                    static_cast<float>(row) *
                        pixelSize;

                glVertex2f(x0, y0);
                glVertex2f(x0 + pixelSize, y0);
                glVertex2f(x0 + pixelSize, y0 + pixelSize);
                glVertex2f(x0, y0 + pixelSize);
            }
        }

        cursorX += 6.0f * pixelSize;
    }

    glEnd();
}

void drawTextCentred(
    float centreX,
    float y,
    float pixelSize,
    const std::string& text,
    Colour colour) {

    drawText(
        centreX - textWidth(text, pixelSize) * 0.5f,
        y,
        pixelSize,
        text,
        colour);
}

std::string formatNumber(
    double value,
    int precision) {

    std::ostringstream stream;

    stream
        << std::fixed
        << std::setprecision(precision)
        << value;

    return stream.str();
}

Colour radarColour(float intensity) {
    const float t =
        std::clamp(intensity, 0.0f, 1.0f);

    if (t < 0.18f) {
        const float u = t / 0.18f;
        return {0.0f, 0.12f + 0.45f * u, 0.20f, 0.35f + 0.35f * u};
    }

    if (t < 0.42f) {
        const float u = (t - 0.18f) / 0.24f;
        return {0.0f + 0.7f * u, 0.62f + 0.28f * u, 0.05f, 0.70f};
    }

    if (t < 0.72f) {
        const float u = (t - 0.42f) / 0.30f;
        return {0.70f + 0.30f * u, 0.90f - 0.30f * u, 0.0f, 0.82f};
    }

    if (t < 0.92f) {
        const float u = (t - 0.72f) / 0.20f;
        return {1.0f, 0.60f - 0.48f * u, 0.0f, 0.90f};
    }

    // const float u = (t - 0.92f) / 0.08f;
    // return {1.0f, 0.12f + 0.88f * u, 0.05f + 0.95f * u, 1.0f};
    return {
        1.0f,  // Red
        0.0f,  // Green
        0.0f,  // Blue
        1.0f   // Alpha
    };
}

void drawPixelCircle(
    float centreX,
    float centreY,
    float radius,
    Colour colour,
    float lineWidth = 1.0f,
    int segmentCount = 256) {

    glLineWidth(lineWidth);
    glColor4f(colour.r, colour.g, colour.b, colour.a);

    glBegin(GL_LINE_LOOP);

    for (int segment = 0;
         segment < segmentCount;
         ++segment) {

        const double angle =
            2.0 * PI *
            static_cast<double>(segment) /
            static_cast<double>(segmentCount);

        glVertex2f(
            centreX +
                radius *
                static_cast<float>(std::cos(angle)),
            centreY +
                radius *
                static_cast<float>(std::sin(angle)));
    }

    glEnd();
    glLineWidth(1.0f);
}

std::vector<PpiTarget> detectDisplayTargets(
    const std::vector<PpiPoint>& points,
    double displayRangeM,
    std::size_t maximumTargetCount) {

    struct Cluster {
        double weightedX{0.0};
        double weightedY{0.0};
        double weight{0.0};
        double peak{0.0};
        int count{0};
    };

    constexpr double CELL_SIZE = 0.055;
    std::map<std::pair<int, int>, Cluster> clusters;

    for (const PpiPoint& point : points) {
        if (point.intensity < 0.62f ||
            point.rangeM < 3.0f) {

            continue;
        }

        const int cellX =
            static_cast<int>(
                std::floor(
                    point.x / CELL_SIZE));

        const int cellY =
            static_cast<int>(
                std::floor(
                    point.y / CELL_SIZE));

        Cluster& cluster =
            clusters[{cellX, cellY}];

        const double weight =
            static_cast<double>(
                point.intensity *
                point.intensity);

        cluster.weightedX +=
            static_cast<double>(point.x) *
            weight;

        cluster.weightedY +=
            static_cast<double>(point.y) *
            weight;

        cluster.weight += weight;

        cluster.peak =
            std::max(
                cluster.peak,
                static_cast<double>(
                    point.intensity));

        ++cluster.count;
    }

    struct Candidate {
        float x;
        float y;
        float score;
    };

    std::vector<Candidate> candidates;

    for (const auto& item : clusters) {
        const Cluster& cluster = item.second;

        if (cluster.weight <= 0.0 ||
            cluster.count < 2) {

            continue;
        }

        candidates.push_back({
            static_cast<float>(
                cluster.weightedX /
                cluster.weight),
            static_cast<float>(
                cluster.weightedY /
                cluster.weight),
            static_cast<float>(
                cluster.peak +
                0.015 *
                    std::min(cluster.count, 20))
        });
    }

    std::sort(
        candidates.begin(),
        candidates.end(),
        [](const Candidate& a,
           const Candidate& b) {
            return a.score > b.score;
        });

    std::vector<PpiTarget> targets;

    for (const Candidate& candidate : candidates) {
        bool closeToExistingTarget = false;

        for (const PpiTarget& target : targets) {
            const float dx = candidate.x - target.x;
            const float dy = candidate.y - target.y;

            if (dx * dx + dy * dy <
                0.115f * 0.115f) {

                closeToExistingTarget = true;
                break;
            }
        }

        if (closeToExistingTarget) {
            continue;
        }

        const float normalisedRange =
            std::sqrt(
                candidate.x * candidate.x +
                candidate.y * candidate.y);

        float bearingDeg =
            static_cast<float>(
                std::atan2(
                    candidate.x,
                    candidate.y) *
                180.0 /
                PI);

        if (bearingDeg < 0.0f) {
            bearingDeg += 360.0f;
        }

        targets.push_back({
            static_cast<int>(targets.size()) + 1,
            candidate.x,
            candidate.y,
            std::clamp(candidate.score, 0.0f, 1.0f),
            normalisedRange *
                static_cast<float>(displayRangeM),
            bearingDeg
        });

        if (targets.size() >=
            maximumTargetCount) {

            break;
        }
    }

    std::sort(
        targets.begin(),
        targets.end(),
        [](const PpiTarget& a,
           const PpiTarget& b) {
            return a.rangeM < b.rangeM;
        });

    for (std::size_t index = 0;
         index < targets.size();
         ++index) {

        targets[index].id =
            static_cast<int>(index) + 1;
    }

    return targets;
}

void drawPpiGrid(
    float centreX,
    float centreY,
    float radius,
    double displayRangeM) {

    const Colour grid{0.05f, 0.55f, 0.10f, 0.52f};
    const Colour fineGrid{0.03f, 0.32f, 0.08f, 0.45f};
    const Colour text{0.70f, 0.82f, 0.70f, 0.95f};

    drawPixelCircle(
        centreX,
        centreY,
        radius,
        {0.12f, 0.95f, 0.18f, 0.88f},
        1.5f);

    for (int ring = 1; ring <= 4; ++ring) {
        drawPixelCircle(
            centreX,
            centreY,
            radius *
                static_cast<float>(ring) /
                4.0f,
            ring == 4 ? grid : fineGrid,
            ring == 4 ? 1.3f : 1.0f);
    }

    glLineWidth(1.0f);

    for (int bearing = 0;
         bearing < 360;
         bearing += 30) {

        const double angle =
            static_cast<double>(bearing) *
            PI /
            180.0;

        const float x =
            centreX +
            radius *
            static_cast<float>(std::sin(angle));

        const float y =
            centreY -
            radius *
            static_cast<float>(std::cos(angle));

        glColor4f(
            fineGrid.r,
            fineGrid.g,
            fineGrid.b,
            fineGrid.a);

        glBegin(GL_LINES);
        glVertex2f(centreX, centreY);
        glVertex2f(x, y);
        glEnd();
    }

    for (int bearing = 0;
         bearing < 360;
         bearing += 5) {

        const double angle =
            static_cast<double>(bearing) *
            PI /
            180.0;

        const float tickLength =
            bearing % 30 == 0 ?
                12.0f :
                (bearing % 10 == 0 ? 8.0f : 4.0f);

        const float outerX =
            centreX +
            radius *
            static_cast<float>(std::sin(angle));

        const float outerY =
            centreY -
            radius *
            static_cast<float>(std::cos(angle));

        const float innerX =
            centreX +
            (radius - tickLength) *
            static_cast<float>(std::sin(angle));

        const float innerY =
            centreY -
            (radius - tickLength) *
            static_cast<float>(std::cos(angle));

        glColor4f(
            grid.r,
            grid.g,
            grid.b,
            grid.a);

        glBegin(GL_LINES);
        glVertex2f(innerX, innerY);
        glVertex2f(outerX, outerY);
        glEnd();
    }

    for (int bearing = 0;
         bearing < 360;
         bearing += 30) {

        const double angle =
            static_cast<double>(bearing) *
            PI /
            180.0;

        const float labelRadius =
            radius + 20.0f;

        const float labelX =
            centreX +
            labelRadius *
            static_cast<float>(std::sin(angle));

        const float labelY =
            centreY -
            labelRadius *
            static_cast<float>(std::cos(angle));

        std::ostringstream label;
        label << std::setw(3) << std::setfill('0') << bearing;

        drawTextCentred(
            labelX,
            labelY - 5.0f,
            1.5f,
            label.str(),
            text);
    }

    for (int ring = 1; ring <= 4; ++ring) {
        const double ringRange =
            displayRangeM *
            static_cast<double>(ring) /
            4.0;

        drawText(
            centreX + 5.0f,
            centreY -
                radius *
                static_cast<float>(ring) /
                4.0f +
                4.0f,
            1.35f,
            formatNumber(ringRange, 0) + " M",
            {0.35f, 0.85f, 0.40f, 0.85f});
    }
}

std::uint32_t hash32(std::uint32_t value) {
    value ^= value >> 16;
    value *= 0x7feb352du;
    value ^= value >> 15;
    value *= 0x846ca68bu;
    value ^= value >> 16;
    return value;
}

float hashUnit(std::uint32_t value) {
    return static_cast<float>(
        hash32(value) & 0x00ffffffu) /
        static_cast<float>(0x01000000u);
}

void drawDisplaySeaTexture(
    float centreX,
    float centreY,
    float radius,
    double timeSeconds,
    const PpiUiState& ui) {

    if (!ui.visualSeaTexture ||
        ui.seaClutter <= 0.0f) {

        return;
    }

    const int pointCount =
        900 +
        static_cast<int>(
            2400.0f *
            ui.seaClutter);

    glPointSize(1.5f);
    glBegin(GL_POINTS);

    for (int index = 0;
         index < pointCount;
         ++index) {

        const float u =
            hashUnit(
                static_cast<std::uint32_t>(
                    index * 11 + 19));

        const float v =
            hashUnit(
                static_cast<std::uint32_t>(
                    index * 23 + 71));

        const float radiusNormalised =
            std::sqrt(u);

        if (radiusNormalised < 0.08f) {
            continue;
        }

        const float angle =
            2.0f *
            static_cast<float>(PI) *
            v;

        const float twinkle =
            0.5f +
            0.5f *
            std::sin(
                static_cast<float>(timeSeconds) *
                    (0.8f + 1.6f * u) +
                static_cast<float>(index));

        const float nearField =
            1.0f -
            std::clamp(
                radiusNormalised,
                0.0f,
                1.0f);

        const float strength =
            ui.seaClutter *
            (0.18f + 0.75f * nearField) *
            (0.25f + 0.75f * twinkle);

        glColor4f(
            0.0f,
            0.18f + 0.40f * strength,
            0.24f + 0.32f * strength,
            0.15f + 0.32f * strength);

        glVertex2f(
            centreX +
                radius *
                radiusNormalised *
                std::cos(angle),
            centreY +
                radius *
                radiusNormalised *
                std::sin(angle));
    }

    glEnd();
}

void drawPpiReturns(
    const std::vector<PpiPoint>& points,
    std::size_t currentAzimuthIndex,
    std::size_t azimuthCount,
    float centreX,
    float centreY,
    float radius,
    const PpiUiState& ui) {

    if (azimuthCount == 0) {
        return;
    }

    auto renderPass =
        [&](float minimumIntensity,
            float pointSize,
            float alphaMultiplier) {

            glPointSize(pointSize);
            glBegin(GL_POINTS);

            for (const PpiPoint& point : points) {
                if (point.intensity <
                    minimumIntensity) {

                    continue;
                }

                const std::size_t age =
                    (currentAzimuthIndex +
                     azimuthCount -
                     point.azimuthIndex) %
                    azimuthCount;

                const float ageFraction =
                    static_cast<float>(age) /
                    static_cast<float>(
                        azimuthCount);

                const float persistenceScale =
                    std::max(
                        0.06f,
                        ui.persistence);

                const float fade =
                    0.22f +
                    0.78f *
                    std::exp(
                        -ageFraction /
                        persistenceScale);

                const float adjusted =
                    std::pow(
                        std::clamp(
                            point.intensity *
                            (0.65f +
                             0.70f *
                             ui.gain),
                            0.0f,
                            1.0f),
                        0.78f);

                Colour colour =
                    radarColour(adjusted);

                colour.a *=
                    fade *
                    alphaMultiplier;

                glColor4f(
                    colour.r,
                    colour.g,
                    colour.b,
                    colour.a);

                glVertex2f(
                    centreX +
                        point.x *
                        radius,
                    centreY -
                        point.y *
                        radius);
            }

            glEnd();
        };

    renderPass(0.00f, 2.0f, 0.82f);
    renderPass(0.58f, 3.5f, 0.42f);
    renderPass(0.82f, 5.5f, 0.32f);
}

void drawPpiSweep(
    double azimuthRad,
    float centreX,
    float centreY,
    float radius) {

    constexpr int TRAIL_COUNT = 38;
    constexpr double TRAIL_SPACING_RAD = 0.0045;

    glLineWidth(1.4f);
    glBegin(GL_LINES);

    for (int trailIndex = 0;
         trailIndex < TRAIL_COUNT;
         ++trailIndex) {

        const double trailAngle =
            azimuthRad -
            static_cast<double>(trailIndex) *
                TRAIL_SPACING_RAD;

        const float brightness =
            1.0f -
            static_cast<float>(trailIndex) /
            static_cast<float>(TRAIL_COUNT);

        glColor4f(
            0.05f,
            1.0f,
            0.22f,
            0.05f +
                0.48f *
                brightness *
                brightness);

        glVertex2f(
            centreX,
            centreY);

        glVertex2f(
            centreX +
                radius *
                static_cast<float>(
                    std::sin(trailAngle)),
            centreY -
                radius *
                static_cast<float>(
                    std::cos(trailAngle)));
    }

    glEnd();
    glLineWidth(1.0f);
}

void drawOwnShip(
    float centreX,
    float centreY,
    float radius) {

    const float shipLength =
        std::max(18.0f, radius * 0.052f);

    const float shipWidth =
        shipLength * 0.42f;

    glLineWidth(2.0f);
    glColor4f(0.15f, 0.75f, 1.0f, 1.0f);

    glBegin(GL_LINE_LOOP);
    glVertex2f(centreX, centreY - shipLength * 0.62f);
    glVertex2f(centreX + shipWidth * 0.5f, centreY - shipLength * 0.10f);
    glVertex2f(centreX + shipWidth * 0.42f, centreY + shipLength * 0.52f);
    glVertex2f(centreX - shipWidth * 0.42f, centreY + shipLength * 0.52f);
    glVertex2f(centreX - shipWidth * 0.5f, centreY - shipLength * 0.10f);
    glEnd();

    glBegin(GL_LINES);
    glVertex2f(centreX, centreY - shipLength * 0.62f);
    glVertex2f(centreX, centreY - radius);
    glEnd();

    drawPixelCircle(
        centreX,
        centreY,
        shipWidth * 0.18f,
        {0.20f, 0.80f, 1.0f, 0.95f},
        1.0f,
        32);

    glLineWidth(1.0f);
}

void drawTargetBracket(
    float x,
    float y,
    float size,
    Colour colour) {

    const float corner = size * 0.34f;

    glColor4f(colour.r, colour.g, colour.b, colour.a);
    glLineWidth(1.8f);
    glBegin(GL_LINES);

    glVertex2f(x - size, y - size);
    glVertex2f(x - size + corner, y - size);
    glVertex2f(x - size, y - size);
    glVertex2f(x - size, y - size + corner);

    glVertex2f(x + size, y - size);
    glVertex2f(x + size - corner, y - size);
    glVertex2f(x + size, y - size);
    glVertex2f(x + size, y - size + corner);

    glVertex2f(x - size, y + size);
    glVertex2f(x - size + corner, y + size);
    glVertex2f(x - size, y + size);
    glVertex2f(x - size, y + size - corner);

    glVertex2f(x + size, y + size);
    glVertex2f(x + size - corner, y + size);
    glVertex2f(x + size, y + size);
    glVertex2f(x + size, y + size - corner);

    glEnd();
    glLineWidth(1.0f);
}

void drawTargets(
    const std::vector<PpiTarget>& targets,
    float centreX,
    float centreY,
    float radius,
    bool enabled) {

    if (!enabled) {
        return;
    }

    for (const PpiTarget& target : targets) {
        const float x =
            centreX +
            target.x * radius;

        const float y =
            centreY -
            target.y * radius;

        const bool nearest =
            target.id == 1;

        const Colour colour =
            nearest ?
                Colour{1.0f, 0.16f, 0.08f, 1.0f} :
                Colour{0.10f, 1.0f, 0.28f, 1.0f};

        drawTargetBracket(
            x,
            y,
            nearest ? 14.0f : 11.0f,
            colour);

        drawText(
            x + 16.0f,
            y - 4.0f,
            1.6f,
            std::to_string(target.id),
            colour);

        if (nearest) {
            drawPixelCircle(
                x,
                y,
                5.0f,
                colour,
                2.0f,
                24);

            glColor4f(
                1.0f,
                0.18f,
                0.08f,
                0.45f);

            glBegin(GL_LINES);
            glVertex2f(centreX, centreY);
            glVertex2f(x, y);
            glEnd();
        }
    }
}

void drawControlBox(
    float x,
    float y,
    float width,
    float height,
    const std::string& label,
    const std::string& value,
    Colour valueColour) {

    drawFilledRect(
        x,
        y,
        width,
        height,
        {0.015f, 0.025f, 0.022f, 0.96f});

    drawRect(
        x,
        y,
        width,
        height,
        {0.18f, 0.30f, 0.25f, 0.95f});

    drawText(
        x + 10.0f,
        y + 8.0f,
        1.35f,
        label,
        {0.18f, 1.0f, 0.28f, 0.95f});

    drawTextCentred(
        x + width * 0.5f,
        y + height * 0.48f,
        2.1f,
        value,
        valueColour);
}

void drawTopStatus(
    int width,
    float leftPanelWidth,
    float rightPanelWidth,
    double displayRangeM,
    double rotationRateHz) {

    constexpr float TOP_HEIGHT = 58.0f;

    drawFilledRect(
        0.0f,
        0.0f,
        static_cast<float>(width),
        TOP_HEIGHT,
        {0.008f, 0.015f, 0.014f, 1.0f});

    drawRect(
        0.0f,
        0.0f,
        static_cast<float>(width),
        TOP_HEIGHT,
        {0.12f, 0.34f, 0.24f, 0.95f});

    drawText(
        18.0f,
        12.0f,
        1.6f,
        "RANGE",
        {0.15f, 1.0f, 0.25f, 1.0f});

    drawText(
        18.0f,
        32.0f,
        2.1f,
        formatNumber(displayRangeM, 0) + " M",
        {0.18f, 1.0f, 0.28f, 1.0f});

    const float centreStart =
        leftPanelWidth;

    const float centreEnd =
        static_cast<float>(width) -
        rightPanelWidth;

    drawTextCentred(
        (centreStart + centreEnd) * 0.5f,
        13.0f,
        2.0f,
        "MAS10 HIGH RESOLUTION MARINE PPI",
        {0.22f, 1.0f, 0.30f, 1.0f});

    drawTextCentred(
        (centreStart + centreEnd) * 0.5f,
        36.0f,
        1.3f,
        "H-UP    RELATIVE MOTION    SCAN " +
            formatNumber(rotationRateHz, 1) +
            " HZ",
        {0.60f, 0.82f, 0.65f, 0.95f});

    drawText(
        static_cast<float>(width) -
            rightPanelWidth +
            18.0f,
        11.0f,
        1.45f,
        "HDG 000.0 DEG",
        {0.20f, 1.0f, 0.28f, 1.0f});

    drawText(
        static_cast<float>(width) -
            rightPanelWidth +
            18.0f,
        33.0f,
        1.45f,
        "SPD 0.0 KN",
        {0.20f, 1.0f, 0.28f, 1.0f});
}

void drawLeftPanel(
    float width,
    int height,
    const PpiUiState& ui) {

    constexpr float TOP_HEIGHT = 58.0f;
    constexpr float BOTTOM_HEIGHT = 72.0f;
    constexpr float MARGIN = 10.0f;

    drawFilledRect(
        0.0f,
        TOP_HEIGHT,
        width,
        static_cast<float>(height) -
            TOP_HEIGHT -
            BOTTOM_HEIGHT,
        {0.008f, 0.014f, 0.013f, 0.98f});

    float y = TOP_HEIGHT + MARGIN;
    const float boxWidth = width - 2.0f * MARGIN;
    constexpr float boxHeight = 70.0f;

    drawControlBox(
        MARGIN,
        y,
        boxWidth,
        boxHeight,
        "GAIN",
        formatNumber(ui.gain * 100.0, 0),
        {0.18f, 1.0f, 0.28f, 1.0f});

    y += boxHeight + 8.0f;

    drawControlBox(
        MARGIN,
        y,
        boxWidth,
        boxHeight,
        "SEA",
        formatNumber(ui.seaClutter * 100.0, 0),
        {0.18f, 1.0f, 0.28f, 1.0f});

    y += boxHeight + 8.0f;

    drawControlBox(
        MARGIN,
        y,
        boxWidth,
        boxHeight,
        "RAIN",
        formatNumber(ui.rainClutter * 100.0, 0),
        {0.18f, 1.0f, 0.28f, 1.0f});

    y += boxHeight + 8.0f;

    drawControlBox(
        MARGIN,
        y,
        boxWidth,
        boxHeight,
        "PERSIST",
        formatNumber(ui.persistence * 100.0, 0),
        {0.18f, 1.0f, 0.28f, 1.0f});

    y += boxHeight + 8.0f;

    drawControlBox(
        MARGIN,
        y,
        boxWidth,
        boxHeight,
        "PALETTE",
        "MULTI",
        {1.0f, 0.72f, 0.05f, 1.0f});

    const float legendY =
        static_cast<float>(height) -
        BOTTOM_HEIGHT -
        178.0f;

    drawText(
        MARGIN,
        legendY,
        1.25f,
        "ECHO STRENGTH",
        {0.62f, 0.78f, 0.65f, 0.95f});

    constexpr int LEGEND_STEPS = 90;

    for (int step = 0;
         step < LEGEND_STEPS;
         ++step) {

        const float t =
            static_cast<float>(step) /
            static_cast<float>(LEGEND_STEPS - 1);

        const Colour colour = radarColour(t);

        drawFilledRect(
            MARGIN,
            legendY + 22.0f +
                (1.0f - t) * 125.0f,
            18.0f,
            2.0f,
            colour);
    }

    drawText(
        MARGIN + 28.0f,
        legendY + 20.0f,
        1.1f,
        "HIGH",
        {0.78f, 0.82f, 0.78f, 0.9f});

    drawText(
        MARGIN + 28.0f,
        legendY + 136.0f,
        1.1f,
        "LOW",
        {0.78f, 0.82f, 0.78f, 0.9f});
}

void drawRightPanel(
    float x,
    float width,
    int height,
    const std::vector<PpiTarget>& targets,
    std::size_t azimuthCount,
    double rangeBinSizeM) {

    constexpr float TOP_HEIGHT = 58.0f;
    constexpr float BOTTOM_HEIGHT = 72.0f;
    constexpr float MARGIN = 10.0f;

    drawFilledRect(
        x,
        TOP_HEIGHT,
        width,
        static_cast<float>(height) -
            TOP_HEIGHT -
            BOTTOM_HEIGHT,
        {0.008f, 0.014f, 0.013f, 0.99f});

    const float innerX = x + MARGIN;
    const float innerWidth = width - 2.0f * MARGIN;
    float y = TOP_HEIGHT + MARGIN;

    drawRect(
        innerX,
        y,
        innerWidth,
        245.0f,
        {0.17f, 0.29f, 0.24f, 1.0f});

    drawTextCentred(
        innerX + innerWidth * 0.5f,
        y + 10.0f,
        1.55f,
        "TARGET LIST",
        {0.85f, 0.88f, 0.85f, 1.0f});

    drawText(
        innerX + 10.0f,
        y + 36.0f,
        1.15f,
        "ID   BRG     RNG     STR",
        {0.60f, 0.72f, 0.64f, 1.0f});

    float rowY = y + 62.0f;

    for (std::size_t index = 0;
         index < std::min<std::size_t>(
             targets.size(),
             6);
         ++index) {

        const PpiTarget& target =
            targets[index];

        std::ostringstream row;
        row
            << std::setw(2)
            << target.id
            << "  "
            << std::setw(5)
            << std::setfill('0')
            << std::fixed
            << std::setprecision(1)
            << target.bearingDeg
            << "  "
            << std::setfill(' ')
            << std::setw(5)
            << std::setprecision(0)
            << target.rangeM
            << "M  "
            << std::setw(3)
            << static_cast<int>(
                target.strength *
                100.0f);

        drawText(
            innerX + 10.0f,
            rowY,
            1.12f,
            row.str(),
            index == 0 ?
                Colour{1.0f, 0.24f, 0.10f, 1.0f} :
                Colour{0.78f, 0.84f, 0.78f, 1.0f});

        rowY += 28.0f;
    }

    if (targets.empty()) {
        drawText(
            innerX + 10.0f,
            rowY,
            1.2f,
            "NO STRONG TARGETS",
            {0.58f, 0.66f, 0.60f, 1.0f});
    }

    y += 257.0f;

    drawRect(
        innerX,
        y,
        innerWidth,
        142.0f,
        {0.17f, 0.29f, 0.24f, 1.0f});

    drawText(
        innerX + 10.0f,
        y + 10.0f,
        1.45f,
        "SCAN INFORMATION",
        {0.82f, 0.86f, 0.82f, 1.0f});

    drawText(
        innerX + 10.0f,
        y + 42.0f,
        1.16f,
        "AZIMUTHS " +
            std::to_string(azimuthCount),
        {0.64f, 0.78f, 0.66f, 1.0f});

    drawText(
        innerX + 10.0f,
        y + 68.0f,
        1.16f,
        "AZ STEP " +
            formatNumber(
                azimuthCount > 0 ?
                    360.0 /
                        static_cast<double>(
                            azimuthCount) :
                    0.0,
                2) +
            " DEG",
        {0.64f, 0.78f, 0.66f, 1.0f});

    drawText(
        innerX + 10.0f,
        y + 94.0f,
        1.16f,
        "RANGE BIN " +
            formatNumber(
                rangeBinSizeM,
                3) +
            " M",
        {0.64f, 0.78f, 0.66f, 1.0f});

    y += 154.0f;

    const float alarmHeight =
        static_cast<float>(height) -
        BOTTOM_HEIGHT -
        y -
        MARGIN;

    drawRect(
        innerX,
        y,
        innerWidth,
        alarmHeight,
        {0.72f, 0.08f, 0.05f, 1.0f},
        1.6f);

    drawText(
        innerX + 10.0f,
        y + 10.0f,
        1.55f,
        "ALARMS",
        {1.0f, 0.16f, 0.08f, 1.0f});

    if (!targets.empty()) {
        const PpiTarget& nearest =
            targets.front();

        drawText(
            innerX + 10.0f,
            y + 44.0f,
            1.25f,
            "NEAREST TARGET",
            {1.0f, 0.20f, 0.08f, 1.0f});

        drawText(
            innerX + 10.0f,
            y + 72.0f,
            1.2f,
            "ID " +
                std::to_string(nearest.id),
            {1.0f, 0.22f, 0.10f, 1.0f});

        drawText(
            innerX + 10.0f,
            y + 98.0f,
            1.2f,
            "BRG " +
                formatNumber(
                    nearest.bearingDeg,
                    1) +
                " DEG",
            {1.0f, 0.22f, 0.10f, 1.0f});

        drawText(
            innerX + 10.0f,
            y + 124.0f,
            1.2f,
            "RNG " +
                formatNumber(
                    nearest.rangeM,
                    1) +
                " M",
            {1.0f, 0.22f, 0.10f, 1.0f});
    }
    else {
        drawText(
            innerX + 10.0f,
            y + 48.0f,
            1.2f,
            "NO ACTIVE ALARMS",
            {0.48f, 0.70f, 0.52f, 1.0f});
    }
}

void drawBottomBar(
    int width,
    int height,
    float leftPanelWidth,
    float rightPanelWidth) {

    constexpr float BOTTOM_HEIGHT = 72.0f;
    const float y =
        static_cast<float>(height) -
        BOTTOM_HEIGHT;

    drawFilledRect(
        0.0f,
        y,
        static_cast<float>(width),
        BOTTOM_HEIGHT,
        {0.008f, 0.015f, 0.014f, 1.0f});

    drawRect(
        0.0f,
        y,
        static_cast<float>(width),
        BOTTOM_HEIGHT,
        {0.14f, 0.28f, 0.22f, 1.0f});

    const std::vector<std::string> buttons{
        "MENU",
        "TARGET",
        "TRAILS ON",
        "SEA AUTO",
        "RAIN OFF",
        "MARK",
        "EVENT"
    };

    const float startX =
        leftPanelWidth + 10.0f;

    const float endX =
        static_cast<float>(width) -
        rightPanelWidth -
        10.0f;

    const float gap = 7.0f;

    const float buttonWidth =
        (endX - startX -
         gap *
             static_cast<float>(buttons.size() - 1)) /
        static_cast<float>(buttons.size());

    for (std::size_t index = 0;
         index < buttons.size();
         ++index) {

        const float x =
            startX +
            static_cast<float>(index) *
                (buttonWidth + gap);

        drawFilledRect(
            x,
            y + 10.0f,
            buttonWidth,
            BOTTOM_HEIGHT - 20.0f,
            {0.015f, 0.025f, 0.022f, 1.0f});

        drawRect(
            x,
            y + 10.0f,
            buttonWidth,
            BOTTOM_HEIGHT - 20.0f,
            {0.18f, 0.32f, 0.26f, 1.0f});

        drawTextCentred(
            x + buttonWidth * 0.5f,
            y + 28.0f,
            1.2f,
            buttons[index],
            index == 2 ?
                Colour{0.18f, 1.0f, 0.28f, 1.0f} :
                Colour{0.72f, 0.78f, 0.73f, 1.0f});
    }
}


struct RadarSelectorLayout
{
    float x{0.0f};
    float y{0.0f};
    float width{0.0f};
    float headerHeight{38.0f};
    float rowHeight{32.0f};
    std::size_t maximumRows{18};
};

RadarSelectorLayout MakeRadarSelectorLayout(
    int width,
    int height)
{
    (void)height;

    const float leftPanelWidth =
        std::clamp(
            static_cast<float>(width) * 0.13f,
            135.0f,
            180.0f);

    const float rightPanelWidth =
        std::clamp(
            static_cast<float>(width) * 0.23f,
            245.0f,
            330.0f);

    const float centreWidth =
        static_cast<float>(width) -
        leftPanelWidth -
        rightPanelWidth;

    RadarSelectorLayout layout;
    layout.x = leftPanelWidth + 12.0f;
    layout.y = 66.0f;
    layout.width =
        std::clamp(
            centreWidth * 0.46f,
            260.0f,
            410.0f);

    return layout;
}

bool PointInside(
    double x,
    double y,
    float left,
    float top,
    float width,
    float height)
{
    return
        x >= left &&
        x <= left + width &&
        y >= top &&
        y <= top + height;
}

bool EndsWith(
    const std::string& value,
    const std::string& suffix)
{
    return
        value.size() >= suffix.size() &&
        value.compare(
            value.size() - suffix.size(),
            suffix.size(),
            suffix) == 0;
}

std::string RadarDisplayName(
    const std::string& topic)
{
    std::vector<std::string> parts;
    std::string current;

    for (const char character : topic) {
        if (character == '/') {
            if (!current.empty()) {
                parts.push_back(current);
                current.clear();
            }
        }
        else {
            current.push_back(character);
        }
    }

    if (!current.empty()) {
        parts.push_back(current);
    }

    if (!parts.empty() && parts.back() == "returns") {
        parts.pop_back();
    }

    std::string label;

    if (parts.size() >= 2) {
        label =
            parts[parts.size() - 2] +
            " / " +
            parts.back();
    }
    else if (!parts.empty()) {
        label = parts.back();
    }
    else {
        label = "NO RADAR SELECTED";
    }

    constexpr std::size_t maximumCharacters = 42;

    if (label.size() > maximumCharacters) {
        label.resize(maximumCharacters - 3);
        label += "...";
    }

    return label;
}

void DrawRadarSelector(
    int width,
    int height,
    const std::vector<std::string>& topics,
    const std::string& selectedTopic,
    bool open)
{
    const RadarSelectorLayout layout =
        MakeRadarSelectorLayout(width, height);

    drawFilledRect(
        layout.x,
        layout.y,
        layout.width,
        layout.headerHeight,
        {0.010f, 0.030f, 0.022f, 0.98f});

    drawRect(
        layout.x,
        layout.y,
        layout.width,
        layout.headerHeight,
        {0.18f, 0.65f, 0.34f, 1.0f},
        1.5f);

    drawText(
        layout.x + 10.0f,
        layout.y + 6.0f,
        1.05f,
        "RADAR SOURCE",
        {0.54f, 0.82f, 0.62f, 1.0f});

    const std::string label =
        selectedTopic.empty() ?
            "NO RADARS FOUND" :
            RadarDisplayName(selectedTopic);

    drawText(
        layout.x + 10.0f,
        layout.y + 20.0f,
        1.25f,
        label,
        selectedTopic.empty() ?
            Colour{1.0f, 0.55f, 0.12f, 1.0f} :
            Colour{0.18f, 1.0f, 0.28f, 1.0f});

    drawText(
        layout.x + layout.width - 20.0f,
        layout.y + 17.0f,
        1.25f,
        open ? "-" : "V",
        {0.18f, 1.0f, 0.28f, 1.0f});

    if (!open) {
        return;
    }

    const std::size_t rowCount =
        std::min(
            topics.size(),
            layout.maximumRows);

    const float listY =
        layout.y +
        layout.headerHeight +
        4.0f;

    if (rowCount == 0) {
        drawFilledRect(
            layout.x,
            listY,
            layout.width,
            layout.rowHeight,
            {0.018f, 0.028f, 0.025f, 0.99f});

        drawRect(
            layout.x,
            listY,
            layout.width,
            layout.rowHeight,
            {0.18f, 0.32f, 0.26f, 1.0f});

        drawText(
            layout.x + 10.0f,
            listY + 10.0f,
            1.15f,
            "WAITING FOR RADAR TOPICS",
            {0.75f, 0.78f, 0.75f, 1.0f});

        return;
    }

    for (std::size_t index = 0;
         index < rowCount;
         ++index)
    {
        const float rowY =
            listY +
            static_cast<float>(index) *
                layout.rowHeight;

        const bool selected =
            topics[index] == selectedTopic;

        drawFilledRect(
            layout.x,
            rowY,
            layout.width,
            layout.rowHeight,
            selected ?
                Colour{0.02f, 0.16f, 0.08f, 0.99f} :
                Colour{0.018f, 0.028f, 0.025f, 0.99f});

        drawRect(
            layout.x,
            rowY,
            layout.width,
            layout.rowHeight,
            {0.18f, 0.32f, 0.26f, 1.0f});

        drawText(
            layout.x + 10.0f,
            rowY + 10.0f,
            1.15f,
            RadarDisplayName(topics[index]),
            selected ?
                Colour{0.18f, 1.0f, 0.28f, 1.0f} :
                Colour{0.75f, 0.80f, 0.76f, 1.0f});
    }
}

void drawPpiWindow(
    GLFWwindow* window,
    const std::vector<PpiPoint>& points,
    const std::vector<PpiTarget>& targets,
    std::size_t currentAzimuthIndex,
    double currentAzimuthRad,
    std::size_t azimuthCount,
    double displayRangeM,
    double rangeBinSizeM,
    double rotationRateHz,
    const PpiUiState& ui,
    double timeSeconds,
    const std::vector<std::string>& radarTopics,
    const std::string& selectedRadarTopic,
    bool radarSelectorOpen) {

    glfwMakeContextCurrent(window);

    int width = 0;
    int height = 0;

    glfwGetFramebufferSize(
        window,
        &width,
        &height);

    if (width <= 0 || height <= 0) {
        return;
    }

    glViewport(0, 0, width, height);
    glDisable(GL_DEPTH_TEST);

    glEnable(GL_BLEND);
    glBlendFunc(
        GL_SRC_ALPHA,
        GL_ONE_MINUS_SRC_ALPHA);

    glEnable(GL_POINT_SMOOTH);
    glHint(
        GL_POINT_SMOOTH_HINT,
        GL_NICEST);

    glClearColor(
        0.002f,
        0.006f,
        0.005f,
        1.0f);

    glClear(GL_COLOR_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glOrtho(
        0.0,
        static_cast<double>(width),
        static_cast<double>(height),
        0.0,
        -1.0,
        1.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    const float leftPanelWidth =
        std::clamp(
            static_cast<float>(width) * 0.13f,
            135.0f,
            180.0f);

    const float rightPanelWidth =
        std::clamp(
            static_cast<float>(width) * 0.23f,
            245.0f,
            330.0f);

    constexpr float TOP_HEIGHT = 58.0f;
    constexpr float BOTTOM_HEIGHT = 72.0f;

    const float centreAreaLeft =
        leftPanelWidth;

    const float centreAreaRight =
        static_cast<float>(width) -
        rightPanelWidth;

    /*
     * Reserve space between the radar-source selector and
     * the top of the circular PPI display.
     */
    constexpr float RADAR_SELECTOR_GAP = 24.0f;

    const float centreAreaTop =
        TOP_HEIGHT +
        RADAR_SELECTOR_GAP;

    const float centreAreaBottom =
        static_cast<float>(height) -
        BOTTOM_HEIGHT;

    const float centreAreaWidth =
        centreAreaRight -
        centreAreaLeft;

    const float centreAreaHeight =
        centreAreaBottom -
        centreAreaTop;

    const float centreX =
        centreAreaLeft +
        centreAreaWidth * 0.5f;

    const float centreY =
        centreAreaTop +
        centreAreaHeight * 0.5f;

    const float radius =
        std::max(
            80.0f,
            std::min(
                centreAreaWidth,
                centreAreaHeight) *
                0.43f);

    drawTopStatus(
        width,
        leftPanelWidth,
        rightPanelWidth,
        displayRangeM,
        rotationRateHz);

    drawLeftPanel(
        leftPanelWidth,
        height,
        ui);

    drawRightPanel(
        static_cast<float>(width) -
            rightPanelWidth,
        rightPanelWidth,
        height,
        targets,
        azimuthCount,
        rangeBinSizeM);

    drawBottomBar(
        width,
        height,
        leftPanelWidth,
        rightPanelWidth);

    drawPixelCircle(
        centreX,
        centreY,
        radius + 3.0f,
        {0.02f, 0.12f, 0.07f, 1.0f},
        7.0f);

    drawDisplaySeaTexture(
        centreX,
        centreY,
        radius,
        timeSeconds,
        ui);

    drawPpiGrid(
        centreX,
        centreY,
        radius,
        displayRangeM);

    drawPpiReturns(
        points,
        currentAzimuthIndex,
        azimuthCount,
        centreX,
        centreY,
        radius,
        ui);

    drawPpiSweep(
        currentAzimuthRad,
        centreX,
        centreY,
        radius);

    drawTargets(
        targets,
        centreX,
        centreY,
        radius,
        ui.targetOverlay);

    drawOwnShip(
        centreX,
        centreY,
        radius);

    drawText(
        centreAreaLeft + 12.0f,
        centreAreaBottom - 24.0f,
        1.2f,
        "DISPLAY SEA TEXTURE IS VISUAL ONLY",
        {0.38f, 0.58f, 0.44f, 0.75f});

    DrawRadarSelector(
        width,
        height,
        radarTopics,
        selectedRadarTopic,
        radarSelectorOpen);

    glfwSwapBuffers(window);
}


class MarinePpiNode final : public rclcpp::Node
{
public:
    MarinePpiNode()
        : rclcpp::Node("fmcw_radar_marine_ppi")
    {
        requestedTopic_ =
            declare_parameter<std::string>(
                "topic",
                "");

        displayRangeM_ =
            declare_parameter<double>(
                "display_range_m",
                1000.0);

        if (!requestedTopic_.empty()) {
            SelectRadarTopic(requestedTopic_);
        }

        RCLCPP_INFO(
            get_logger(),
            "Marine PPI radar discovery enabled");
    }

    void RefreshRadarTopics()
    {
        const auto now =
            std::chrono::steady_clock::now();

        if (lastDiscoveryTime_ !=
                std::chrono::steady_clock::time_point{} &&
            now - lastDiscoveryTime_ <
                std::chrono::seconds(1))
        {
            return;
        }

        lastDiscoveryTime_ = now;

        std::vector<std::string> discovered;

        const auto graph =
            get_topic_names_and_types();

        for (const auto& entry : graph) {
            const std::string& topic = entry.first;
            const std::vector<std::string>& types =
                entry.second;

            if (!EndsWith(topic, "/returns")) {
                continue;
            }

            const bool correctType =
                std::find(
                    types.begin(),
                    types.end(),
                    "std_msgs/msg/Float32MultiArray") !=
                types.end();

            if (correctType) {
                discovered.push_back(topic);
            }
        }

        std::sort(
            discovered.begin(),
            discovered.end());

        discovered.erase(
            std::unique(
                discovered.begin(),
                discovered.end()),
            discovered.end());

        availableTopics_ =
            std::move(discovered);

        if (selectedTopic_.empty() &&
            !availableTopics_.empty())
        {
            SelectRadarTopic(
                availableTopics_.front());
        }
    }

    void HandleMouseClick(
        double x,
        double y,
        int framebufferWidth,
        int framebufferHeight)
    {
        const RadarSelectorLayout layout =
            MakeRadarSelectorLayout(
                framebufferWidth,
                framebufferHeight);

        if (PointInside(
                x,
                y,
                layout.x,
                layout.y,
                layout.width,
                layout.headerHeight))
        {
            selectorOpen_ = !selectorOpen_;
            return;
        }

        if (!selectorOpen_) {
            return;
        }

        const float listY =
            layout.y +
            layout.headerHeight +
            4.0f;

        const std::size_t rowCount =
            std::min(
                availableTopics_.size(),
                layout.maximumRows);

        for (std::size_t index = 0;
             index < rowCount;
             ++index)
        {
            const float rowY =
                listY +
                static_cast<float>(index) *
                    layout.rowHeight;

            if (PointInside(
                    x,
                    y,
                    layout.x,
                    rowY,
                    layout.width,
                    layout.rowHeight))
            {
                SelectRadarTopic(
                    availableTopics_[index]);

                selectorOpen_ = false;
                return;
            }
        }

        selectorOpen_ = false;
    }

    const std::vector<PpiPoint>& Points() const
    {
        return points_;
    }

    const std::vector<PpiTarget>& Targets() const
    {
        return targets_;
    }

    std::size_t AzimuthCount() const
    {
        return std::max<std::size_t>(1, azimuthCount_);
    }

    double DisplayRangeM() const
    {
        return displayRangeM_;
    }

    double RangeBinSizeM() const
    {
        return rangeBinSizeM_;
    }

    double RotationRateHz() const
    {
        return rotationRateHz_;
    }

    const PpiUiState& Ui() const
    {
        return ui_;
    }

    const std::vector<std::string>&
    AvailableRadarTopics() const
    {
        return availableTopics_;
    }

    const std::string&
    SelectedRadarTopic() const
    {
        return selectedTopic_;
    }

    bool RadarSelectorOpen() const
    {
        return selectorOpen_;
    }

private:
    void SelectRadarTopic(
        const std::string& topic)
    {
        if (topic.empty()) {
            return;
        }

        if (topic == selectedTopic_ &&
            subscription_)
        {
            return;
        }

        subscription_.reset();
        ClearRadarData();

        selectedTopic_ = topic;

        subscription_ =
            create_subscription<
                std_msgs::msg::Float32MultiArray>(
                selectedTopic_,
                rclcpp::SensorDataQoS(),
                [this](
                    const std_msgs::msg::Float32MultiArray::ConstSharedPtr message)
                {
                    OnReturns(message);
                });

        RCLCPP_INFO(
            get_logger(),
            "Marine PPI selected radar [%s]",
            selectedTopic_.c_str());
    }

    void ClearRadarData()
    {
        points_.clear();
        targets_.clear();
        azimuthCount_ = 400;
        rangeBinSizeM_ = 0.155;
        rotationRateHz_ = 2.0;
    }

    void OnReturns(
        const std_msgs::msg::Float32MultiArray::ConstSharedPtr& message)
    {
        if (!message || message->data.size() < 4) {
            return;
        }

        const std::size_t payloadCount =
            message->data.size() - 4;

        if ((payloadCount % 4) != 0) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Malformed FMCW return packet");
            return;
        }

        azimuthCount_ =
            std::max<std::size_t>(
                1,
                static_cast<std::size_t>(
                    std::llround(message->data[0])));

        rangeBinSizeM_ =
            std::max(
                1e-6,
                static_cast<double>(message->data[1]));

        const double sensorMaximumRangeM =
            std::max(
                rangeBinSizeM_,
                static_cast<double>(message->data[2]));

        rotationRateHz_ =
            std::max(
                1e-6,
                static_cast<double>(message->data[3]));

        displayRangeM_ =
            std::clamp(
                displayRangeM_,
                rangeBinSizeM_,
                sensorMaximumRangeM);

        double maximumPowerW = 0.0;

        for (std::size_t index = 4;
             index + 3 < message->data.size();
             index += 4)
        {
            const double powerW =
                static_cast<double>(message->data[index + 2]);

            if (powerW > maximumPowerW &&
                std::isfinite(powerW))
            {
                maximumPowerW = powerW;
            }
        }

        points_.clear();

        if (!(maximumPowerW > 0.0)) {
            targets_.clear();
            return;
        }

        points_.reserve(payloadCount / 4);

        for (std::size_t index = 4;
             index + 3 < message->data.size();
             index += 4)
        {
            const double azimuthRad =
                static_cast<double>(message->data[index]);

            const double rangeM =
                static_cast<double>(message->data[index + 1]);

            const double powerW =
                static_cast<double>(message->data[index + 2]);

            const std::size_t azimuthIndex =
                static_cast<std::size_t>(
                    std::max(
                        0.0f,
                        message->data[index + 3]));

            if (!(powerW > 0.0) ||
                !std::isfinite(powerW) ||
                rangeM < 0.0 ||
                rangeM > displayRangeM_)
            {
                continue;
            }

            const double relativeDb =
                10.0 *
                std::log10(
                    powerW / maximumPowerW);

            if (relativeDb <
                -PPI_DYNAMIC_RANGE_DB)
            {
                continue;
            }

            const float intensity =
                static_cast<float>(
                    std::clamp(
                        1.0 +
                            relativeDb /
                                PPI_DYNAMIC_RANGE_DB,
                        0.0,
                        1.0));

            const double radius =
                rangeM / displayRangeM_;

            points_.push_back({
                static_cast<float>(
                    radius *
                    std::sin(azimuthRad)),
                static_cast<float>(
                    radius *
                    std::cos(azimuthRad)),
                intensity,
                static_cast<float>(rangeM),
                azimuthIndex,
                static_cast<std::size_t>(
                    std::floor(
                        rangeM / rangeBinSizeM_))
            });
        }

        targets_ =
            detectDisplayTargets(
                points_,
                displayRangeM_,
                6);
    }

    std::string requestedTopic_;
    std::string selectedTopic_;

    std::vector<std::string>
        availableTopics_;

    bool selectorOpen_{false};

    std::chrono::steady_clock::time_point
        lastDiscoveryTime_{};

    double displayRangeM_{1000.0};
    double rangeBinSizeM_{0.155};
    double rotationRateHz_{2.0};
    std::size_t azimuthCount_{400};

    PpiUiState ui_;

    std::vector<PpiPoint> points_;
    std::vector<PpiTarget> targets_;

    rclcpp::Subscription<
        std_msgs::msg::Float32MultiArray>::SharedPtr
        subscription_;
};

} // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<MarinePpiNode>();

    if (!glfwInit()) {
        std::cerr << "GLFW initialization failed\n";
        rclcpp::shutdown();
        return 1;
    }

    GLFWwindow* window =
        glfwCreateWindow(
            1160,
            820,
            "MAS10 Detailed Marine PPI",
            nullptr,
            nullptr);

    if (!window) {
        glfwTerminate();
        rclcpp::shutdown();
        return 1;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    bool previousMouseDown = false;

    while (rclcpp::ok() &&
           !glfwWindowShouldClose(window))
    {
        rclcpp::spin_some(node);
        node->RefreshRadarTopics();

        const bool mouseDown =
            glfwGetMouseButton(
                window,
                GLFW_MOUSE_BUTTON_LEFT) ==
            GLFW_PRESS;

        if (mouseDown && !previousMouseDown) {
            double cursorX = 0.0;
            double cursorY = 0.0;

            glfwGetCursorPos(
                window,
                &cursorX,
                &cursorY);

            int windowWidth = 1;
            int windowHeight = 1;
            int framebufferWidth = 1;
            int framebufferHeight = 1;

            glfwGetWindowSize(
                window,
                &windowWidth,
                &windowHeight);

            glfwGetFramebufferSize(
                window,
                &framebufferWidth,
                &framebufferHeight);

            cursorX *=
                static_cast<double>(framebufferWidth) /
                static_cast<double>(
                    std::max(1, windowWidth));

            cursorY *=
                static_cast<double>(framebufferHeight) /
                static_cast<double>(
                    std::max(1, windowHeight));

            node->HandleMouseClick(
                cursorX,
                cursorY,
                framebufferWidth,
                framebufferHeight);
        }

        previousMouseDown = mouseDown;

        const double now =
            glfwGetTime();

        const std::size_t azimuthCount =
            node->AzimuthCount();

        const double rotations =
            now * node->RotationRateHz();

        const double rotationFraction =
            rotations - std::floor(rotations);

        const std::size_t azimuthIndex =
            std::min(
                static_cast<std::size_t>(
                    rotationFraction *
                    static_cast<double>(azimuthCount)),
                azimuthCount - 1);

        const double azimuthRad =
            2.0 * PI *
            static_cast<double>(azimuthIndex) /
            static_cast<double>(azimuthCount);

        drawPpiWindow(
            window,
            node->Points(),
            node->Targets(),
            azimuthIndex,
            azimuthRad,
            azimuthCount,
            node->DisplayRangeM(),
            node->RangeBinSizeM(),
            node->RotationRateHz(),
            node->Ui(),
            now,
            node->AvailableRadarTopics(),
            node->SelectedRadarTopic(),
            node->RadarSelectorOpen());

        if (glfwGetKey(window, GLFW_KEY_ESCAPE) ==
            GLFW_PRESS)
        {
            glfwSetWindowShouldClose(window, true);
        }

        glfwPollEvents();
    }

    glfwDestroyWindow(window);
    glfwTerminate();

    rclcpp::shutdown();
    return 0;
}
