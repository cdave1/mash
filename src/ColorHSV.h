#ifndef _MATH_SCRAPS_COLOR_HSV_
#define _MATH_SCRAPS_COLOR_HSV_

#include <Color4f.h>

class ColorHSV {
public:
    ColorHSV() : hue(0.0f), saturation(1.0f), value(1.0f), alpha(1.0f) {
    }


    ColorHSV(float _hue, float _saturation, float _value, float _alpha) :
        hue(_hue), saturation(_saturation), value(_value), alpha(_alpha) {}


    void SetHue(float _hue) {
        hue = scraps::clamp(_hue, 0.0f, 1.0f);
    }


    void SetSaturation(float _saturation) {
        saturation = scraps::clamp(_saturation, 0.0f, 1.0f);
    }


    void SetValue(float _value) {
        value = scraps::clamp(_value, 0.0f, 360.0f);
    }


    void SetAlpha(float _alpha) {
        alpha = scraps::clamp(_alpha, 0.0f, 1.0f);
    }


    void Set(const Color4f &color) {

    }


    Color4f ToColor() {
        return Color4f(1.0f, 1.0f, 1.0f, 1.0f);
    }


private:
    float hue;
    float saturation;
    float value;
    float alpha;
}

#endif
