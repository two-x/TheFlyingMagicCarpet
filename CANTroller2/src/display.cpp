#include "display.h"

Display::Display(int8_t cs_pin, int8_t dc_pin)
    : _tft(cs_pin, dc_pin),
      _tftResetTimer(),
      _tftDelayTimer(),
      _timing_tft_reset(),
      _procrastinate(),
      reset_finished(),
      _disp_redraw_all(){}

void Display::init()
{
    printf("Init LCD... ");
    yield();
    _tft.begin();
    _tft.setRotation((flip_the_screen) ? 3 : 1); // 0: Portrait, USB Top-Rt, 1: Landscape, usb=Bot-Rt, 2: Portrait, USB=Bot-Rt, 3: Landscape, USB=Top-Lt
    for (int32_t lineno = 0; lineno <= disp_fixed_lines; lineno++)
    {
        disp_age_quanta[lineno] = -1;
        memset(disp_values[lineno], 0, strlen(disp_values[lineno]));
        disp_polarities[lineno] = 1;
    }
    for (int32_t row = 0; row < arraysize(disp_bool_values); row++)
        disp_bool_values[row] = 1;
    for (int32_t row = 0; row < arraysize(disp_needles); row++)
        disp_needles[row] = -5; // Otherwise the very first needle draw will blackout a needle shape at x=0. Do this offscreen
    for (int32_t row = 0; row < arraysize(disp_targets); row++)
        disp_targets[row] = -5; // Otherwise the very first target draw will blackout a target shape at x=0. Do this offscreen
    yield();
    _tft.fillScreen(BLK); // Black out the whole screen
    yield();
    draw_touchgrid(false);
    yield();
    draw_fixed(dataset_page, dataset_page_last, false);
    yield();
    _disp_redraw_all = true;
    printf("Success.\nTouchscreen initialization... ");
    ts.begin();
    // if (!ts.begin(40)) printf ("Couldn't start touchscreen controller");  // pass in 'sensitivity' coefficient
    printf("Touchscreen started\n");
}
bool Display::tft_reset()
{ // call to begin a tft reset, and continue to call every loop until returns true (or get_reset_finished() returns true), then stop
    if (reset_finished)
    {
        reset_finished = false;
        _timing_tft_reset = 1;
    }
    if (_timing_tft_reset == 1)
    {
        write_pin(tft_rst_pin, LOW);
        _timing_tft_reset = 2;
    }
    else if (_timing_tft_reset == 2 && _tftResetTimer.expired())
    {
        write_pin(tft_rst_pin, HIGH);
        init();
        _timing_tft_reset = 0;
        reset_finished = true;
    }
    return reset_finished;
}
void Display::watchdog()
{ // Call in every loop to perform a reset upon detection of blocked loops and
    if (loop_period_us > tft_watchdog_timeout_us && _timing_tft_reset == 0)
        _timing_tft_reset = 1;
    if (_timing_tft_reset == 0 || !_tftDelayTimer.expired())
        _tftDelayTimer.reset();
    else
        tft_reset();
}
bool Display::get_reset_finished() { return reset_finished; }

// Functions to write to the screen efficiently
//
void Display::draw_bargraph_base(int32_t corner_x, int32_t corner_y, int32_t width)
{ // draws a horizontal bargraph scale.  124, y, 40
    _tft.drawFastHLine(corner_x + disp_bargraph_squeeze, corner_y, width - disp_bargraph_squeeze * 2, GRY1);
    for (int32_t offset = 0; offset <= 2; offset++)
        _tft.drawFastVLine((corner_x + disp_bargraph_squeeze) + offset * (width / 2 - disp_bargraph_squeeze), corner_y - 1, 3, WHT);
}
void Display::draw_needle_shape(int32_t pos_x, int32_t pos_y, int32_t color)
{ // draws a cute little pointy needle
    _tft.drawFastVLine(pos_x - 1, pos_y, 2, color);
    _tft.drawFastVLine(pos_x, pos_y, 4, color);
    _tft.drawFastVLine(pos_x + 1, pos_y, 2, color);
}
void Display::draw_target_shape(int32_t pos_x, int32_t pos_y, int32_t t_color, int32_t r_color)
{ // draws a cute little target symbol
    _tft.drawFastVLine(pos_x - 1, pos_y + 7, 2, t_color);
    _tft.drawFastVLine(pos_x, pos_y + 5, 4, t_color);
    _tft.drawFastVLine(pos_x + 1, pos_y + 7, 2, t_color);
}
void Display::draw_bargraph_needle(int32_t n_pos_x, int32_t old_n_pos_x, int32_t pos_y, int32_t n_color)
{ // draws a cute little pointy needle
    draw_needle_shape(old_n_pos_x, pos_y, BLK);
    draw_needle_shape(n_pos_x, pos_y, n_color);
}
void Display::draw_string(int32_t x_new, int32_t x_old, int32_t y, const char *text, const char *oldtext, int32_t color, int32_t bgcolor, bool forced)
{ // Send in "" for oldtext if erase isn't needed
    int32_t oldlen = strlen(oldtext);
    int32_t newlen = strlen(text);
    _tft.setTextColor(bgcolor);
    for (int32_t letter = 0; letter < oldlen; letter++)
    {
        if (newlen - letter < 1)
        {
            _tft.setCursor(x_old + disp_font_width * letter, y);
            _tft.print(oldtext[letter]);
        }
        else if (oldtext[letter] != text[letter])
        {
            _tft.setCursor(x_old + disp_font_width * letter, y);
            _tft.print(oldtext[letter]);
        }
    }
    _tft.setTextColor(color);
    for (int32_t letter = 0; letter < newlen; letter++)
    {
        if (oldlen - letter < 1)
        {
            _tft.setCursor(x_new + disp_font_width * letter, y);
            _tft.print(text[letter]);
        }
        else if (oldtext[letter] != text[letter] || forced)
        {
            _tft.setCursor(x_new + disp_font_width * letter, y);
            _tft.print(text[letter]);
        }
    }
}
void Display::draw_mmph(int32_t x, int32_t y, int32_t color)
{ // This is my cheesy pixel-drawn "mmph" compressed horizontally to 3-char width
    _tft.setTextColor(color);
    _tft.setCursor(x, y);
    _tft.print("m");
    _tft.setCursor(x + 4, y);
    _tft.print("m"); // Overlapping 'mm' complete (x = 0-8)
    _tft.drawFastVLine(x + 10, y + 2, 6, color);
    _tft.drawPixel(x + 11, y + 2, color);
    _tft.drawPixel(x + 11, y + 6, color);
    _tft.drawFastVLine(x + 12, y + 3, 3, color); // 'p' complete (x = 10-12)
    _tft.drawFastVLine(x + 14, y, 7, color);
    _tft.drawPixel(x + 15, y + 2, color);
    _tft.drawFastVLine(x + 16, y + 3, 4, color); // 'h' complete (x = 14-16)
}
void Display::draw_thou(int32_t x, int32_t y, int32_t color)
{ // This is my cheesy pixel-drawn "thou" compressed horizontally to 3-char width
    _tft.drawFastVLine(x + 1, y + 1, 5, color);
    _tft.drawFastHLine(x, y + 2, 3, color);
    _tft.drawPixel(x + 2, y + 6, color); // 't' complete (x = 0-2)
    _tft.drawFastVLine(x + 4, y, 7, color);
    _tft.drawPixel(x + 5, y + 3, color);
    _tft.drawPixel(x + 6, y + 2, color);
    _tft.drawFastVLine(x + 7, y + 3, 4, color); // 'h' complete (x = 4-7)
    _tft.drawFastVLine(x + 9, y + 3, 3, color);
    _tft.drawFastHLine(x + 10, y + 2, 2, color);
    _tft.drawFastHLine(x + 10, y + 6, 2, color);
    _tft.drawFastVLine(x + 12, y + 3, 3, color); // 'o' complete (x = 9-12)
    _tft.drawFastVLine(x + 14, y + 2, 4, color);
    _tft.drawPixel(x + 15, y + 6, color);
    _tft.drawFastVLine(x + 16, y + 2, 5, color); // 'u' complete (x = 14-16)
}
void Display::draw_string_units(int32_t x, int32_t y, const char *text, const char *oldtext, int32_t color, int32_t bgcolor)
{ // Send in "" for oldtext if erase isn't needed
    _tft.setCursor(x, y);
    _tft.setTextColor(bgcolor);
    _tft.print(oldtext); // Erase the old content
    _tft.setCursor(x, y);
    _tft.setTextColor(color);
    _tft.print(text); // Erase the old content
}
void Display::draw_colons(int32_t x_pos, int32_t first, int32_t last, int32_t color)
{
    for (int32_t lineno = first; lineno <= last; lineno++)
    {
        _tft.drawPixel(x_pos, lineno * disp_line_height_pix + 3, color); // Tiny microscopic colon dots
        _tft.drawPixel(x_pos, lineno * disp_line_height_pix + 7, color); // Tiny microscopic colon dots
        // _tft.fillRect (x_pos, (lineno+1)*disp_line_height_pix+3, 2, 2, color);  // Big goofy looking colon dots
        // _tft.fillRect (x_pos, (lineno+1)*disp_line_height_pix+7, 2, 2, color);  // Big goofy looking colon dots
    }
}
// draw_fixed displays 20 rows of text strings with variable names. and also a column of text indicating units, plus boolean names, all in grey.
void Display::draw_fixed(int32_t page, int32_t page_last, bool redraw_tuning_corner, bool forced)
{            // set redraw_tuning_corner to true in order to just erase the tuning section and redraw
    yield(); // experiment
    _tft.setTextColor(GRY2);
    _tft.setTextSize(1);
    // if (redraw_tuning_corner) _tft.fillRect(10, 145, 154, 95, BLK); // _tft.fillRect(0,145,167,95,BLK);  // Erase old dataset page area - This line alone uses 15 ms
    int32_t y_pos;
    if (!redraw_tuning_corner)
    {
        for (int32_t lineno = 0; lineno < disp_fixed_lines; lineno++)
        { // Step thru lines of fixed telemetry data
            y_pos = (lineno + 1) * disp_line_height_pix + disp_vshift_pix;
            draw_string(12, 12, y_pos, telemetry[lineno], "", GRY2, BLK, forced);
            draw_string_units(104, y_pos, units[lineno], "", GRY2, BLK);
            draw_bargraph_base(124, y_pos + 7, disp_bargraph_width);
        }
        // draw_colons(7+disp_font_width*arraysize(telemetry[0]), 1, disp_fixed_lines+disp_tuning_lines, GRY1);  // I can't decide if I like the colons or not
    }
    for (int32_t lineno = 0; lineno < disp_tuning_lines; lineno++)
    {            // Step thru lines of dataset page data
        yield(); // experiment
        draw_string(12, 12, (lineno + disp_fixed_lines + 1) * disp_line_height_pix + disp_vshift_pix, dataset_page_names[page][lineno], dataset_page_names[page_last][lineno], GRY2, BLK, forced);
        draw_string_units(104, (lineno + disp_fixed_lines + 1) * disp_line_height_pix + disp_vshift_pix, tuneunits[page][lineno], tuneunits[page_last][lineno], GRY2, BLK);
        if (redraw_tuning_corner)
        {
            int32_t corner_y = (lineno + disp_fixed_lines + 1) * disp_line_height_pix + disp_vshift_pix + 7; // lineno*disp_line_height_pix+disp_vshift_pix-1;
            draw_bargraph_base(124, corner_y, disp_bargraph_width);
            if (disp_needles[lineno] >= 0)
                draw_bargraph_needle(-1, disp_needles[lineno], corner_y - 6, BLK); // Let's draw a needle
        }
    }
}
void Display::draw_hyphen(int32_t x_pos, int32_t y_pos, int32_t color)
{ // Draw minus sign in front of negative numbers
    _tft.drawFastHLine(x_pos + 2, y_pos + 3, 3, color);
}
void Display::draw_dynamic(int32_t lineno, char const *disp_string, int32_t value, int32_t lowlim, int32_t hilim, int32_t target)
{
    yield();                                                                       // experiment
    int32_t age_us = (int32_t)((float)(dispAgeTimer[lineno].elapsed()) / 2500000); // Divide by us per color gradient quantum
    int32_t x_base = 59;
    bool polarity = (value >= 0); // polarity 0=negative, 1=positive
    if (strcmp(disp_values[lineno], disp_string) || value == 1234567 || _disp_redraw_all)
    { // If value differs, Erase old value and write new
        int32_t y_pos = lineno * disp_line_height_pix + disp_vshift_pix;
        if (polarity != disp_polarities[lineno])
            draw_hyphen(x_base, y_pos, (!polarity) ? GRN : BLK);
        draw_string(x_base + disp_font_width, x_base + disp_font_width, y_pos, disp_string, disp_values[lineno], GRN, BLK); // +6*(arraysize(modecard[runmode])+4-namelen)/2
        strcpy(disp_values[lineno], disp_string);
        disp_polarities[lineno] = polarity;
        dispAgeTimer[lineno].reset();
        disp_age_quanta[lineno] = 0;
    } // to-do: Fix failure to freshen aged coloration of unchanged characters of changed values
    else if (age_us > disp_age_quanta[lineno] && age_us < 11)
    { // As readings age, redraw in new color. This may fail and redraw when the timer overflows?
        int32_t color;
        if (age_us < 8)
            color = 0x1fe0 + age_us * 0x2000; // Base of green with red added as you age, until yellow is achieved
        else
            color = 0xffe0 - (age_us - 8) * 0x100; // Then lose green as you age further
        int32_t y_pos = (lineno)*disp_line_height_pix + disp_vshift_pix;
        if (!polarity)
            draw_hyphen(x_base, y_pos, color);
        draw_string(x_base + disp_font_width, x_base + disp_font_width, y_pos, disp_values[lineno], "", color, BLK);
        disp_age_quanta[lineno] = age_us;
    }
    yield(); // experiment
    if (lowlim < hilim)
    { // Any value having a given range deserves a bargraph gauge with a needle
        int32_t corner_x = 124;
        int32_t corner_y = lineno * disp_line_height_pix + disp_vshift_pix - 1;
        int32_t n_pos = map(value, lowlim, hilim, disp_bargraph_squeeze, disp_bargraph_width - disp_bargraph_squeeze);
        int32_t ncolor = (n_pos > disp_bargraph_width - disp_bargraph_squeeze || n_pos < disp_bargraph_squeeze) ? DORG : GRN;
        n_pos = corner_x + constrain(n_pos, disp_bargraph_squeeze, disp_bargraph_width - disp_bargraph_squeeze);
        if (target != -1)
        { // If target value is given, draw a target on the bargraph too
            int32_t t_pos = map(target, lowlim, hilim, disp_bargraph_squeeze, disp_bargraph_width - disp_bargraph_squeeze);
            int32_t tcolor = (t_pos > disp_bargraph_width - disp_bargraph_squeeze || t_pos < disp_bargraph_squeeze) ? DORG : ((t_pos != n_pos) ? YEL : GRN);
            t_pos = corner_x + constrain(t_pos, disp_bargraph_squeeze, disp_bargraph_width - disp_bargraph_squeeze);
            if (t_pos != disp_targets[lineno] || (t_pos == n_pos) ^ (disp_needles[lineno] != disp_targets[lineno]) || _disp_redraw_all)
            {
                draw_target_shape(disp_targets[lineno], corner_y, BLK, -1);                                                                                                                                                                                             // Erase old target
                _tft.drawFastHLine(disp_targets[lineno] - (disp_targets[lineno] != corner_x + disp_bargraph_squeeze), lineno * disp_line_height_pix + disp_vshift_pix + 7, 2 + (disp_targets[lineno] != corner_x + disp_bargraph_width - disp_bargraph_squeeze), GRY1); // Patch bargraph line where old target got erased
                for (int32_t offset = 0; offset <= 2; offset++)
                    _tft.drawFastVLine((corner_x + disp_bargraph_squeeze) + offset * (disp_bargraph_width / 2 - disp_bargraph_squeeze), lineno * disp_line_height_pix + disp_vshift_pix + 6, 3, WHT); // Redraw bargraph graduations in case one got corrupted by target erasure
                draw_target_shape(t_pos, corner_y, tcolor, -1);                                                                                                                                       // Draw the new target
                disp_targets[lineno] = t_pos;                                                                                                                                                         // Remember position of target
            }
        }
        if (n_pos != disp_needles[lineno] || _disp_redraw_all)
        {
            draw_bargraph_needle(n_pos, disp_needles[lineno], corner_y, ncolor); // Let's draw a needle
            disp_needles[lineno] = n_pos;                                        // Remember position of needle
        }
    }
    else if (disp_needles[lineno] >= 0)
    {                                                                                                             // If value having no range is drawn over one that did ...
        draw_bargraph_needle(-1, disp_needles[lineno], lineno * disp_line_height_pix + disp_vshift_pix - 1, BLK); // Erase the old needle
        disp_needles[lineno] = -1;                                                                                // Flag for no needle
    }
}
int32_t Display::significant_place(float value)
{ // Returns the decimal place of the most significant digit of a given float value, without relying on logarithm math
    int32_t place = 0;
    if (value >= 1)
    { // int32_t vallog = std::log10(value);  // Can be sped up
        place = 1;
        while (value >= 10)
        {
            value /= 10;
            place++;
        }
    }
    else if (value)
    { // checking (value) rather than (value != 0.0) can help avoid precision errors caused by digital representation of floating numbers
        while (value < 1)
        {
            value *= 10;
            place--;
        }
    }
    return place;
}
std::string Display::abs_itoa(int32_t value, int32_t maxlength)
{                       // returns an ascii string representation of a given integer value, using scientific notation if necessary to fit within given width constraint
    value = abs(value); // This function disregards sign
    if (significant_place(value) <= maxlength)
        return std::to_string(value); // If value is short enough, return it
    std::string result;
    int32_t magnitude = std::log10(value);                               // check how slow is log() function? Compare performance vs. multiple divides ( see abs_ftoa() )
    float scaledValue = value / std::pow(10, magnitude + 1 - maxlength); // was (10, magnitude - 5);
    if (scaledValue >= 1.0 && scaledValue < 10.0)
        result = std::to_string(static_cast<int>(scaledValue));
    else
        result = std::to_string(scaledValue);
    if (magnitude >= maxlength)
        result += "e" + std::to_string(magnitude);
    return result;
}
std::string Display::abs_ftoa(float value, int32_t maxlength, int32_t sigdig)
{                                             // returns an ascii string representation of a given float value, formatted to efficiently fit withinthe given width constraint
    value = abs(value);                       // This function disregards sign
    int32_t place = significant_place(value); // Learn decimal place of the most significant digit in value
    if (place >= sigdig && place <= maxlength)
    { // Then we want simple cast to an integer w/o decimal point (eg 123456, 12345, 1234)
        std::string result(std::to_string((int32_t)value));
        return result;
    }
    if (place >= 0 && place < maxlength)
    { // Then we want float formatted with enough nonzero digits after the decimal point for 4 significant digits (eg 123.4, 12.34, 1.234, 0)
        int32_t length = min(sigdig + 1, maxlength);
        char buffer[length + 1];
        std::snprintf(buffer, length + 1, "%.*g", length - 1, value);
        std::string result(buffer); // copy buffer to result
        return result;
    }
    if (place >= 3 - maxlength && place < maxlength)
    { // Then we want decimal w/o initial '0' limited to 3 significant digits (eg .123, .0123, .00123)
        std::string result(std::to_string(value));
        size_t decimalPos = result.find('.'); // Remove any digits to the left of the decimal point
        if (decimalPos != std::string::npos)
            result = result.substr(decimalPos);
        if (result.length() > sigdig)
            result.resize(sigdig + 1); // Limit the string length to the desired number of significant digits
        return result;
    }                                                 // Otherwise we want scientific notation with precision removed as needed to respect maxlength (eg 1.23e4, 1.23e5, but using long e character not e for negative exponents
    char buffer[maxlength + 1];                       // Allocate buffer with the maximum required size
    int32_t sigdigless = sigdig - 1 - (place <= -10); // was: if (place <= -10) return std::string ("~0");  // Ridiculously small values just indicate basically zero
    snprintf(buffer, sizeof(buffer), "%*.*f%*d", maxlength - sigdigless, sigdigless, value, maxlength - 1, 0);
    std::string result(buffer); // copy buffer to result
    if (result.find("e+0") != std::string::npos)
        result.replace(result.find("e+0"), 3, "e"); // Remove useless "+0" from exponent
    else if (result.find("e-0") != std::string::npos)
        result.replace(result.find("e-0"), 3, "\x88"); // For very small scientific notation values, replace the "e-0" with a phoenetic long e character, to indicate negative power  // if (result.find ("e-0") != std::string::npos)
    else if (result.find("e+") != std::string::npos)
        result.replace(result.find("e+"), 3, "e"); // For ridiculously large values
    else if (result.find("e-") != std::string::npos)
        result.replace(result.find("e-"), 3, "\x88"); // For ridiculously small values
    return result;
}
void Display::draw_dynamic(int32_t lineno, int32_t value, int32_t lowlim, int32_t hilim, int32_t target)
{
    std::string val_string = abs_itoa(value, (int32_t)disp_maxlength);
    // std::cout << "Int: " << value << " -> " << val_string << ", " << ((value >= 0) ? 1 : -1) << std::endl;
    draw_dynamic(lineno, val_string.c_str(), value, lowlim, hilim, (int32_t)target);
}
void Display::draw_dynamic(int32_t lineno, float value, float lowlim, float hilim, int32_t target)
{
    std::string val_string = abs_ftoa(value, (int32_t)disp_maxlength, 3);
    // std::cout << "Flt: " << value << " -> " << val_string << ", " << ((value >= 0) ? 1 : -1) << std::endl;
    draw_dynamic(lineno, val_string.c_str(), (int32_t)value, (int32_t)lowlim, (int32_t)hilim, target);
}
void Display::draw_dynamic(int32_t lineno, float value, float lowlim, float hilim, float target)
{
    draw_dynamic(lineno, value, lowlim, hilim, (int32_t)target);
}
void Display::draw_dynamic(int32_t lineno, int32_t erasure)
{
    std::string val_string = "";
    if (erasure == ERASE)
        draw_dynamic(lineno, val_string.c_str(), 1234567, -1, -1, -1);
}
void Display::draw_runmode(int32_t runmode, int32_t oldmode, int32_t color_override)
{ // color_override = -1 uses default color
    yield();
    int32_t color = (color_override == -1) ? colorcard[runmode] : color_override;
    int32_t x_new = 8 + 6 * (2 + strlen(modecard[runmode])) - 3;
    int32_t x_old = 8 + 6 * (2 + strlen(modecard[oldmode])) - 3;
    draw_string(8 + 6, 8 + 6, disp_vshift_pix, modecard[oldmode], "", BLK, BLK);   // +6*(arraysize(modecard[runmode])+4-namelen)/2
    draw_string(x_old, x_old, disp_vshift_pix, "Mode", "", BLK, BLK);              // +6*(arraysize(modecard[runmode])+4-namelen)/2
    draw_string(8 + 6, 8 + 6, disp_vshift_pix, modecard[runmode], "", color, BLK); // +6*(arraysize(modecard[runmode])+4-namelen)/2
    draw_string(x_new, x_new, disp_vshift_pix, "Mode", "", color, BLK);            // +6*(arraysize(modecard[runmode])+4-namelen)/2
}
void Display::draw_dataset_page(int32_t page, int32_t page_last, bool forced)
{
    draw_fixed(page, page_last, true, forced); // Erase and redraw dynamic data corner of screen with names, units etc.
    // for (int32_t lineno=0; lineno<disp_lines; lineno++) draw_hyphen (59, lineno*disp_line_height_pix+disp_vshift_pix, BLK);
    yield();
    draw_string(83, 83, disp_vshift_pix, pagecard[page], pagecard[page_last], RBLU, BLK, forced); // +6*(arraysize(modecard[runmode])+4-namelen)/2
}
void Display::draw_selected_name(int32_t tun_ctrl, int32_t tun_ctrl_last, int32_t selected_val, int32_t selected_last)
{
    yield();
    if (selected_val != selected_last)
        draw_string(12, 12, 12 + (selected_last + disp_fixed_lines) * disp_line_height_pix + disp_vshift_pix, dataset_page_names[dataset_page][selected_last], "", GRY2, BLK);
    draw_string(12, 12, 12 + (selected_val + disp_fixed_lines) * disp_line_height_pix + disp_vshift_pix, dataset_page_names[dataset_page][selected_val], "", (tun_ctrl == EDIT) ? GRN : ((tun_ctrl == SELECT) ? YEL : GRY2), BLK);
}
void Display::draw_bool(bool value, int32_t col)
{ // Draws values of boolean data
    if ((disp_bool_values[col - 2] != value) || _disp_redraw_all)
    { // If value differs, Erase old value and write new
        int32_t x_mod = touch_margin_h_pix + touch_cell_h_pix * (col) + (touch_cell_h_pix >> 1) - arraysize(top_menu_buttons[col - 2] - 1) * (disp_font_width >> 1) - 2;
        draw_string(x_mod, x_mod, 0, top_menu_buttons[col - 2], "", (value) ? GRN : LGRY, DGRY);
        disp_bool_values[col - 2] = value;
    }
}
void Display::draw_simbuttons(bool create)
{ // draw grid of buttons to simulate sensors. If create is true it draws buttons, if false it erases them
    _tft.setTextColor(LYEL);
    for (int32_t row = 0; row < arraysize(simgrid); row++)
    {
        for (int32_t col = 0; col < arraysize(simgrid[row]); col++)
        {
            yield();
            int32_t cntr_x = touch_margin_h_pix + touch_cell_h_pix * (col + 3) + (touch_cell_h_pix >> 1) + 2;
            int32_t cntr_y = touch_cell_v_pix * (row + 1) + (touch_cell_v_pix >> 1);
            if (strcmp(simgrid[row][col], "    "))
            {
                _tft.fillCircle(cntr_x, cntr_y, 19, create ? DGRY : BLK);
                if (create)
                {
                    _tft.drawCircle(cntr_x, cntr_y, 19, LYEL);
                    int32_t x_mod = cntr_x - (arraysize(simgrid[row][col]) - 1) * (disp_font_width >> 1);
                    draw_string(x_mod, x_mod, cntr_y - (disp_font_height >> 1), simgrid[row][col], "", LYEL, DGRY);
                }
            }
        }
    }
}
void Display::draw_touchgrid(bool side_only)
{ // draws edge buttons with names in 'em. If replace_names, just updates names
    int32_t namelen = 0;
    _tft.setTextColor(WHT);
    for (int32_t row = 0; row < arraysize(side_menu_buttons); row++)
    { // Step thru all rows to draw buttons along the left edge
        yield();
        _tft.fillRoundRect(-9, touch_cell_v_pix * row + 3, 18, touch_cell_v_pix - 6, 8, DGRY);
        _tft.drawRoundRect(-9, touch_cell_v_pix * row + 3, 18, touch_cell_v_pix - 6, 8, LYEL);
        namelen = 0;
        for (uint32_t x = 0; x < arraysize(side_menu_buttons[row]); x++)
        {
            if (side_menu_buttons[row][x] != ' ')
                namelen++; // Go thru each button name. Need to remove spaces padding the ends of button names shorter than 4 letters
        }
        for (int32_t letter = 0; letter < namelen; letter++)
        { // Going letter by letter thru each button name so we can write vertically
            yield();
            _tft.setCursor(1, (touch_cell_v_pix * row) + (touch_cell_v_pix / 2) - (int32_t)(4.5 * ((float)namelen - 1)) + (disp_font_height + 1) * letter); // adjusts vertical offset depending how many letters in the button name and which letter we're on
            _tft.println(side_menu_buttons[row][letter]);                                                                                                   // Writes each letter such that the whole name is centered vertically on the button
        }
    }
    if (!side_only)
    {
        for (int32_t col = 2; col <= 5; col++)
        { // Step thru all cols to draw buttons across the top edge
            yield();
            _tft.fillRoundRect(touch_margin_h_pix + touch_cell_h_pix * (col) + 3, -9, touch_cell_h_pix - 6, 18, 8, DGRY);
            _tft.drawRoundRect(touch_margin_h_pix + touch_cell_h_pix * (col) + 3, -9, touch_cell_h_pix - 6, 18, 8, LYEL); // _tft.width()-9, 3, 18, (_tft.height()/5)-6, 8, LYEL);
            // draw_bool (top_menu_buttons[btn], btn+3);
        }
    }
}

void Display::update()
{
    if (simulating != simulating_last || _disp_redraw_all)
    {
        draw_simbuttons(simulating); // if we just entered simulator draw the simulator buttons, or if we just left erase them
        simulating_last = simulating;
        _procrastinate = true; // Waits till next loop to draw changed values
    }
    if ((disp_dataset_page_dirty || _disp_redraw_all))
    {
        static bool first = true;
        draw_dataset_page(dataset_page, dataset_page_last, first);
        first = false;
        disp_dataset_page_dirty = false;
        if (dataset_page_last != dataset_page)
            config.putUInt("dpage", dataset_page);
        dataset_page_last = dataset_page;
        _procrastinate = true; // Waits till next loop to draw changed values
    }
    if ((disp_sidemenu_dirty || _disp_redraw_all))
    {
        draw_touchgrid(true);
        disp_sidemenu_dirty = false;
        _procrastinate = true; // Waits till next loop to draw changed values
    }
    if (disp_selected_val_dirty || _disp_redraw_all)
    {
        draw_selected_name(tuning_ctrl, tuning_ctrl_last, selected_value, selected_value_last);
        disp_selected_val_dirty = false;
        selected_value_last = selected_value;
        tuning_ctrl_last = tuning_ctrl; // Make sure this goes after the last comparison
    }
    if (disp_runmode_dirty || _disp_redraw_all)
    {
        draw_runmode(runmode, oldmode, (runmode == SHUTDOWN) ? shutdown_color : -1);
        disp_runmode_dirty = false;
        oldmode = runmode; // remember what mode we're in for next time
    }
    if ((dispRefreshTimer.expired() && !_procrastinate) || _disp_redraw_all)
    {
        dispRefreshTimer.reset();
        float drange;
        draw_dynamic(1, ctrl_pos_adc[VERT][FILT], ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX]);
        draw_dynamic(2, speedo_filt_mph, 0.0, speedo_redline_mph, speedo_target_mph);
        draw_dynamic(3, tach_filt_rpm, 0.0, tach_redline_rpm, tach_target_rpm);
        draw_dynamic(4, gas_pulse_out_us, gas_pulse_redline_us, gas_pulse_idle_us);
        draw_dynamic(5, pressure_filt_psi, pressure_min_psi, pressure_max_psi, pressure_target_psi); // (brake_active_pid == S_PID) ? (int32_t)brakeSPID.get_target() : pressure_target_adc);
        draw_dynamic(6, (int32_t)brake_pulse_out_us, brake_pulse_retract_us, brake_pulse_extend_us);
        draw_dynamic(7, ctrl_pos_adc[HORZ][FILT], ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX]);
        draw_dynamic(8, steer_pulse_out_us, steer_pulse_right_us, steer_pulse_left_us);
        if (dataset_page == PG_RUN)
        {
            draw_dynamic(9, airflow_filt_mph, airflow_min_mph, airflow_max_mph);
            draw_dynamic(10, brake_pos_filt_in, brake_pos_nom_lim_retract_in, brake_pos_nom_lim_extend_in);
            draw_dynamic(11, battery_filt_v, 0.0, battery_max_v);
            draw_dynamic(12, pot_filt_percent, pot_min_percent, pot_max_percent);
            draw_dynamic(13, sim_airflow, -1, -1);
            draw_dynamic(14, sim_brkpos, -1, -1);
            draw_dynamic(15, sim_joy, -1, -1);
            draw_dynamic(16, sim_pressure, -1, -1);
            draw_dynamic(17, sim_tach, -1, -1);
            draw_dynamic(18, sim_speedo, -1, -1);
            draw_dynamic(19, pot_overload, -1, -1);
        }
        else if (dataset_page == PG_JOY)
        {
            draw_dynamic(9, ctrl_pos_adc[HORZ][RAW], ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX]);
            draw_dynamic(10, ctrl_pos_adc[VERT][RAW], ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX]);
            draw_dynamic(11, hotrc_horz_pulse_us, hotrc_pulse_lims_us[HORZ][MIN], hotrc_pulse_lims_us[HORZ][MAX]); // Programmed centerpoint is 230 adc
            draw_dynamic(12, hotrc_vert_pulse_us, hotrc_pulse_lims_us[VERT][MIN], hotrc_pulse_lims_us[VERT][MAX]); // Programmed centerpoint is 230 adc
            draw_dynamic(13, hotrc_pulse_failsafe_max_us, hotrc_pulse_lims_us[VERT][MIN], hotrc_pulse_lims_us[VERT][MAX]);
            draw_dynamic(14, ctrl_lims_adc[ctrl][HORZ][MIN], 0, ctrl_lims_adc[ctrl][HORZ][MAX]);
            draw_dynamic(15, ctrl_lims_adc[ctrl][HORZ][MAX], ctrl_lims_adc[ctrl][HORZ][MIN], adcrange_adc);
            draw_dynamic(16, ctrl_lims_adc[ctrl][HORZ][DB], 0, 2 * (min(ctrl_lims_adc[ctrl][HORZ][CENT] - ctrl_lims_adc[ctrl][HORZ][MIN], ctrl_lims_adc[ctrl][HORZ][MAX] - ctrl_lims_adc[ctrl][HORZ][CENT]) - 1));
            draw_dynamic(17, ctrl_lims_adc[ctrl][VERT][MIN], 0, ctrl_lims_adc[ctrl][VERT][MAX]);
            draw_dynamic(18, ctrl_lims_adc[ctrl][VERT][MAX], ctrl_lims_adc[ctrl][VERT][MIN], adcrange_adc);
            draw_dynamic(19, ctrl_lims_adc[ctrl][VERT][DB], 0, 2 * (min(ctrl_lims_adc[ctrl][VERT][CENT] - ctrl_lims_adc[ctrl][VERT][MIN], ctrl_lims_adc[ctrl][VERT][MAX] - ctrl_lims_adc[ctrl][VERT][CENT]) - 1));
        }
        else if (dataset_page == PG_CAR)
        {
            draw_dynamic(9, pressure_adc, pressure_min_adc, pressure_max_adc);
            draw_dynamic(10, ERASE);
            draw_dynamic(11, ERASE);
            draw_dynamic(12, gas_governor_percent, 0, 100);
            draw_dynamic(13, steer_safe_percent, 0, 100);
            draw_dynamic(14, airflow_max_mph, 0.0, airflow_abs_max_mph);
            draw_dynamic(15, tach_idle_rpm, 0.0, tach_redline_rpm);
            draw_dynamic(16, tach_redline_rpm, 0.0, tach_max_rpm);
            draw_dynamic(17, speedo_idle_mph, 0.0, speedo_redline_mph);
            draw_dynamic(18, speedo_redline_mph, 0.0, speedo_max_mph);
            draw_dynamic(19, brake_pos_zeropoint_in, brake_pos_nom_lim_retract_in, brake_pos_nom_lim_extend_in);
        }
        else if (dataset_page == PG_PWMS)
        {
            draw_dynamic(9, ERASE);
            draw_dynamic(10, ERASE);
            draw_dynamic(11, ERASE);
            draw_dynamic(12, steer_pulse_left_us, steer_pulse_stop_us, steer_pulse_left_max_us);
            draw_dynamic(13, steer_pulse_stop_us, steer_pulse_left_us, steer_pulse_right_us);
            draw_dynamic(14, steer_pulse_right_us, steer_pulse_right_min_us, steer_pulse_stop_us);
            draw_dynamic(15, brake_pulse_extend_us, brake_pulse_stop_us, brake_pulse_extend_max_us);
            draw_dynamic(16, brake_pulse_stop_us, brake_pulse_retract_us, brake_pulse_extend_us);
            draw_dynamic(17, brake_pulse_retract_us, brake_pulse_retract_min_us, brake_pulse_stop_us);
            draw_dynamic(18, gas_pulse_idle_us, gas_pulse_cw_min_us, gas_pulse_ccw_max_us);
            draw_dynamic(19, gas_pulse_redline_us, gas_pulse_cw_min_us, gas_pulse_ccw_max_us);
        }
        else if (dataset_page == PG_BPID)
        {
            drange = brake_pulse_extend_us - brake_pulse_retract_us;
            draw_dynamic(9, pressure_target_psi, pressure_min_psi, pressure_max_psi);
            draw_dynamic(10, brakeQPID.GetError(), pressure_min_psi - pressure_max_psi, pressure_max_psi - pressure_min_psi);
            draw_dynamic(11, brakeQPID.GetPterm(), -drange, drange);
            draw_dynamic(12, brakeQPID.GetIterm(), -drange, drange);
            draw_dynamic(13, brakeQPID.GetDterm(), -drange, drange);
            draw_dynamic(14, brakeQPID.GetOutputSum(), (float)brake_pulse_retract_us, (float)brake_pulse_extend_us); // brake_spid_speedo_delta_adc, -range, range);
            draw_dynamic(15, ERASE);
            draw_dynamic(16, ERASE);
            draw_dynamic(17, brakeQPID.GetKp(), 0.0, 2.0);
            draw_dynamic(18, brakeQPID.GetKi(), 0.0, 2.0);
            draw_dynamic(19, brakeQPID.GetKd(), 0.0, 2.0);
        }
        else if (dataset_page == PG_GPID)
        {
            drange = gas_pulse_idle_us - gas_pulse_govern_us;
            draw_dynamic(9, tach_target_rpm, 0.0, tach_redline_rpm);
            draw_dynamic(10, gasQPID.GetError(), tach_idle_rpm - tach_govern_rpm, tach_govern_rpm - tach_idle_rpm);
            draw_dynamic(11, gasQPID.GetPterm(), -drange, drange);
            draw_dynamic(12, gasQPID.GetIterm(), -drange, drange);
            draw_dynamic(13, gasQPID.GetDterm(), -drange, drange);
            draw_dynamic(14, gasQPID.GetOutputSum(), (float)gas_pulse_idle_us, (float)gas_pulse_govern_us);
            draw_dynamic(15, ERASE);
            draw_dynamic(16, gas_open_loop, -1, -1);
            draw_dynamic(17, gasQPID.GetKp(), 0.0, 2.0);
            draw_dynamic(18, gasQPID.GetKi(), 0.0, 2.0);
            draw_dynamic(19, gasQPID.GetKd(), 0.0, 2.0);
        }
        else if (dataset_page == PG_CPID)
        {
            drange = tach_govern_rpm - tach_idle_rpm;
            draw_dynamic(9, speedo_target_mph, 0.0, speedo_govern_mph);
            draw_dynamic(10, cruiseQPID.GetError(), speedo_idle_mph - speedo_govern_mph, speedo_govern_mph - speedo_idle_mph);
            draw_dynamic(11, cruiseQPID.GetPterm(), -drange, drange);
            draw_dynamic(12, cruiseQPID.GetIterm(), -drange, drange);
            draw_dynamic(13, cruiseQPID.GetDterm(), -drange, drange);
            draw_dynamic(14, cruiseQPID.GetOutputSum(), tach_idle_rpm, tach_govern_rpm); // cruise_spid_speedo_delta_adc, -drange, drange);
            draw_dynamic(15, tach_target_rpm, 0.0, tach_redline_rpm);
            draw_dynamic(16, ERASE);
            draw_dynamic(17, cruiseQPID.GetKp(), 0.0, 2.0);
            draw_dynamic(18, cruiseQPID.GetKi(), 0.0, 2.0);
            draw_dynamic(19, cruiseQPID.GetKd(), 0.0, 2.0);
        }
        else if (dataset_page == PG_TEMP)
        {
            draw_dynamic(9, temps[AMBIENT], temp_min, temp_max);
            draw_dynamic(10, temps[ENGINE], temp_min, temp_max);
            draw_dynamic(11, temps[WHEEL_FL], temp_min, temp_max);
            draw_dynamic(12, temps[WHEEL_FR], temp_min, temp_max);
            draw_dynamic(13, temps[WHEEL_RL], temp_min, temp_max);
            draw_dynamic(14, temps[WHEEL_RR], temp_min, temp_max);
            draw_dynamic(15, ERASE);
            draw_dynamic(16, ERASE);
            draw_dynamic(17, ERASE);
            draw_dynamic(18, cal_joyvert_brkmotor, -1, -1);
            draw_dynamic(19, cal_pot_gasservo, -1, -1);
        }
        draw_bool((runmode == CAL), 2);
        draw_bool((runmode == BASIC), 3);
        draw_bool(ignition, 4);
        draw_bool(syspower, 5);
    }
    _procrastinate = false;
    _disp_redraw_all = false;
}