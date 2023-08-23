#ifndef TEMPERATURELIMITS_H
#define TEMPERATURELIMITS_H

class TemperatureLimits {
private:
    float disp_min;
    float nom_min;
    float nom_max;
    float warning;
    float alarm;
    float disp_max;

public:
    TemperatureLimits(float disp_min, float nom_min, float nom_max, float warning, float alarm, float disp_max)
        : disp_min(disp_min), nom_min(nom_min), nom_max(nom_max), warning(warning), alarm(alarm), disp_max(disp_max) {}

    float get_disp_min() const { return disp_min; }
    float get_nom_min() const { return nom_min; }
    float get_nom_max() const { return nom_max; }
    float get_warning() const { return warning; }
    float get_alarm() const { return alarm; }
    float get_disp_max() const { return disp_max; }
};

#endif // TEMPERATURELIMITS_H