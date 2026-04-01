/**
 *    Led Animation header
 * 
 *    0                  6
 *     1      [] []     7
 *      2              8
 *       3    [] []   9
 *        4          10
 *         5        11
*/

#ifndef UNITREE_ROS2_INTERFACE_FACE_LIGHTS_ANIMATIONS_HPP
#define UNITREE_ROS2_INTERFACE_FACE_LIGHTS_ANIMATIONS_HPP

#include <array>
#include <cstdint>
#include <vector>
#include <unordered_map>

constexpr uint32_t NUM_FACE_LEDS = 12;

/// Singolo frame: colore RGB per ogni LED + durata in millisecondi
struct LedFrame {
    std::array<std::array<uint8_t, 3>, NUM_FACE_LEDS> colors;
    uint32_t duration_ms;
};

/// Animazione: sequenza di frame con flag loop e ID identificativo
struct LedAnimation {
    uint32_t id;
    bool is_loop;
    std::vector<LedFrame> frames;
};

// ──────────────────────────────────────────────
// Helper: crea un frame con tutti i LED dello stesso colore
// ──────────────────────────────────────────────
inline LedFrame make_uniform_frame(uint8_t r, uint8_t g, uint8_t b, uint32_t duration_ms) {
    LedFrame f;
    f.duration_ms = duration_ms;
    for (auto & c : f.colors) { c = {r, g, b}; }
    return f;
}

// ══════════════════════════════════════════════
//  Animazioni predefinite
// ══════════════════════════════════════════════

/// ID 1 – Blink rosso/spento (loop)
inline LedAnimation anim_blink_red() {
    LedAnimation a;
    a.id = 1;
    a.is_loop = true;
    a.frames = {
        make_uniform_frame(255, 0, 0, 500),
        make_uniform_frame(  0, 0, 0, 500),
    };
    return a;
}

/// ID 2 – Ciclo RGB (loop)
inline LedAnimation anim_cycle_rgb() {
    LedAnimation a;
    a.id = 2;
    a.is_loop = true;
    a.frames = {
        make_uniform_frame(255,   0,   0, 400),
        make_uniform_frame(  0, 255,   0, 400),
        make_uniform_frame(  0,   0, 255, 400),
    };
    return a;
}

/// ID 3 – Flash bianco singolo (no loop)
inline LedAnimation anim_flash_white() {
    LedAnimation a;
    a.id = 3;
    a.is_loop = false;
    a.frames = {
        make_uniform_frame(255, 255, 255, 200),
        make_uniform_frame(  0,   0,   0, 200),
        make_uniform_frame(255, 255, 255, 200),
        make_uniform_frame(  0,   0,   0, 200),
    };
    return a;
}

/// ID 4 – Scorrimento laterale blu (loop)
inline LedAnimation anim_sweep_blue() {
    LedAnimation a;
    a.id = 4;
    a.is_loop = true;
    // LED layout: 0-5 lato sinistro (alto→basso), 6-11 lato destro (alto→basso)
    for (uint32_t i = 0; i < NUM_FACE_LEDS; ++i) {
        LedFrame f;
        f.duration_ms = 100;
        for (auto & c : f.colors) { c = {0, 0, 0}; }
        f.colors[i] = {0, 80, 255};
        a.frames.push_back(f);
    }
    return a;
}

/// ID 5 – Rainbow gradiente che scorre dal basso verso l'alto (loop)
inline LedAnimation anim_rainbow_sweep() {
    LedAnimation a;
    a.id = 5;
    a.is_loop = true;

    // 12 colori arcobaleno per transizioni più fluide
    constexpr uint32_t NUM_COLORS = 12;
    constexpr std::array<std::array<uint8_t, 3>, NUM_COLORS> rainbow = {{
        {255,   0,   0},   // rosso
        {255,  80,   0},   // rosso-arancio
        {255, 160,   0},   // arancione
        {255, 255,   0},   // giallo
        {128, 255,   0},   // giallo-verde
        {  0, 255,   0},   // verde
        {  0, 255, 128},   // verde-ciano
        {  0, 255, 255},   // ciano
        {  0, 128, 255},   // ciano-blu
        {  0,   0, 255},   // blu
        {128,   0, 255},   // viola
        {255,   0, 128},   // magenta
    }};

    constexpr uint32_t NUM_ROWS = 6;

    // 12 frame: ogni step shifta il gradiente verso l'alto di una posizione
    // Le 6 righe campionano la palette con passo 2 → coprono tutti i 12 colori in un ciclo
    for (uint32_t step = 0; step < NUM_COLORS; ++step) {
        LedFrame f;
        f.duration_ms = 150;
        for (uint32_t row = 0; row < NUM_ROWS; ++row) {
            uint32_t color_idx = (row * 2 + step) % NUM_COLORS;
            f.colors[row]            = rainbow[color_idx];  // colonna sinistra
            f.colors[row + NUM_ROWS] = rainbow[color_idx];  // colonna destra
        }
        a.frames.push_back(f);
    }

    return a;
}

/// ID 6 – Luci polizia: alterna blu (sinistra) / rosso (destra) con flash (loop)
inline LedAnimation anim_police() {
    LedAnimation a;
    a.id = 6;
    a.is_loop = true;

    constexpr uint32_t NUM_ROWS = 6;
    constexpr std::array<uint8_t, 3> blue = {0, 0, 255};
    constexpr std::array<uint8_t, 3> red  = {255, 0, 0};
    constexpr std::array<uint8_t, 3> off  = {0, 0, 0};

    auto make_police_frame = [&](bool left_on, bool right_on,
                                 const std::array<uint8_t,3>& left_col,
                                 const std::array<uint8_t,3>& right_col,
                                 uint32_t duration_ms) {
        LedFrame f;
        f.duration_ms = duration_ms;
        for (uint32_t row = 0; row < NUM_ROWS; ++row) {
            f.colors[row]            = left_on  ? left_col  : off;
            f.colors[row + NUM_ROWS] = right_on ? right_col : off;
        }
        return f;
    };

    // Blu a sinistra: 2 flash rapidi
    a.frames.push_back(make_police_frame(true,  false, blue, red, 100));
    a.frames.push_back(make_police_frame(false, false, blue, red, 80));

    // Rosso a destra: 2 flash rapidi
    a.frames.push_back(make_police_frame(false, true,  blue, red, 100));
    a.frames.push_back(make_police_frame(false, false, blue, red, 80));

    return a;
}

// ──────────────────────────────────────────────
// Registro globale: mappa id → animazione
// ──────────────────────────────────────────────
inline const std::unordered_map<uint32_t, LedAnimation> & getAnimationRegistry() {
    static const std::unordered_map<uint32_t, LedAnimation> registry = [] {
        std::unordered_map<uint32_t, LedAnimation> m;
        auto add = [&](LedAnimation a) { m[a.id] = std::move(a); };
        add(anim_blink_red());
        add(anim_cycle_rgb());
        add(anim_flash_white());
        add(anim_sweep_blue());
        add(anim_rainbow_sweep());
        add(anim_police());
        return m;
    }();
    return registry;
}

#endif // UNITREE_ROS2_INTERFACE_FACE_LIGHTS_ANIMATIONS_HPP