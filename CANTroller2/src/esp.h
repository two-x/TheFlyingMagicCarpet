#pragma once
#include <esp_task_wdt.h>
#include <iostream>
#include <iomanip>  // For formatting console loop timing string output

class LoopTimer {
  public:
    LoopTimer() {}
    // Loop timing related
    Timer loop_timer = Timer(1000000);  // how long the previous main loop took to run (in us)
    int32_t loopno = 1, loopindex = 0, loop_recentsum = 0, loop_scale_min_us = 0, loop_scale_avg_max_us = 2500, loop_scale_peak_max_us = 25000;
    float loop_sum_s;
    uint32_t looptimes_us[20];
    bool loop_dirty[20];
    int64_t loop_cout_mark_us, boot_mark;
    uint32_t loop_cout_us = 0, loop_now = 0;;
    static constexpr uint32_t loop_history = 100;
    uint32_t loop_periods_us[loop_history];
    // std::vector<std::string> loop_names(20);
    std::string loop_names[20];
    void setup() {  // Run once at end of setup()
        boot_mark = esp_timer_get_time();
        if (looptime_print) {
            for (int32_t x=1; x<arraysize(loop_dirty); x++) loop_dirty[x] = true;
            loop_names[0] = std::string("top");
            loop_dirty[0] = false;
            loopindex = 1;
            looptimes_us[0] = esp_timer_get_time();
        }
        loop_timer.reset();  // start timer to measure the first loop
    }
    void mark(std::string loopname = std::string("")) {  // Add marks wherever you want in the main loop, set looptime_print true, will report times between all adjacent marks
        if (looptime_print) {
            if (loop_dirty[loopindex]) {
                loop_names[loopindex] = loopname;  // names[index], name);
                loop_dirty[loopindex] = false;
            }
            looptimes_us[loopindex] = esp_timer_get_time();
            loopindex++;
        }
    }
    float calc_avg(uint32_t _loop_now, uint32_t _thisloop) {
        if (_loop_now == loop_history + 2) {
            loop_recentsum = _thisloop;
            for (int l = 0; l <= loop_history; l++)
                loop_recentsum += loop_periods_us[(_loop_now + l) % loop_history];
        }
        else loop_recentsum += _thisloop - loop_periods_us[loop_now];
        return (float)loop_recentsum/(float)loop_history;
    }
    void update() {  // Call once each loop at the very end
        uint32_t thisloop = (uint32_t)loop_timer.elapsed();
        loop_avg_us = calc_avg(loop_now, thisloop);
        loop_periods_us[loop_now] = thisloop;  // us since beginning of this loop
        loop_timer.reset();
        loop_sum_s += (float)loop_periods_us[loop_now] / 1000000;
        // ema_filt(loop_periods_us[loop_now], &loop_avg_us, 0.01);
        if (loop_avg_us > 1) loopfreq_hz = 1000000/loop_avg_us;
        loop_peak_us = 0;
        for (int8_t i=0; i<loop_history; i++) if (loop_peak_us < loop_periods_us[i]) loop_peak_us = loop_periods_us[i]; 
        if (looptime_print) {
            loop_cout_mark_us = esp_timer_get_time();
            std::cout << std::fixed << std::setprecision(0);
            std::cout << "\r" << (uint32_t)loop_sum_s << "s #" << loopno;  //  << " av:" << std::setw(5) << (int32_t)(loop_avg_us);  //  << " av:" << std::setw(3) << loop_avg_ms 
            std::cout << " : " << std::setw(5) << loop_periods_us[loop_now] << " (" << std::setw(5) << loop_periods_us[loop_now]-loop_cout_us << ")us ";  // << " avg:" << loop_avg_us;  //  " us:" << esp_timer_get_time() << 
            for (int32_t x=1; x<loopindex; x++)
                std::cout << std::setw(3) << loop_names[x] << ":" << std::setw(5) << looptimes_us[x]-looptimes_us[x-1] << " ";
            std::cout << " cout:" << std::setw(5) << loop_cout_us;
            if (loop_periods_us[loop_now]-loop_cout_us > looptime_linefeed_threshold || !looptime_linefeed_threshold) std::cout << std::endl;
            loop_cout_us = (uint32_t)(esp_timer_get_time() - loop_cout_mark_us);
            loopindex = 0;
            mark ("top");
        }
        uptime();
        ++loop_now %= loop_history;
        loopno++;  // I like to count how many loops
    }
    void uptime() {  // returns uptime since last reset in minutes
        uptime_min = (float)((esp_timer_get_time() - boot_mark)) / (60.0 * 1000000.0);
    }
};
class BootMonitor {
  private:
    int timeout_sec = 10;
    uint32_t uptime_recorded = -1, uptime_rounding = 5;
    Preferences* myprefs;
    LoopTimer* myloop;
    int codestatus_last = 50000, crashcount = 0;
    uint32_t bootcount;                         // variable to track total number of boots of this code build
    uint32_t codestatus_postmortem;
    std::string codestatuscard[NumCodeStatuses] = { "confused", "booting", "parked", "stopped", "driving" };
    Timer highWaterTimer{30000000};
    TaskHandle_t* task1; TaskHandle_t* task2; TaskHandle_t* task3; TaskHandle_t* task4;
    UBaseType_t highWaterBytes;
    bool was_panicked = false;
  public:
    int boot_to_runmode = STANDBY;
    BootMonitor(Preferences* _prefs, LoopTimer* _loop) : myprefs(_prefs), myloop(_loop) {}
    void set_codestatus(int _mode) {
        codestatus = _mode;
        if (codestatus_last != codestatus) myprefs->putUInt("codestatus", codestatus);
        codestatus_last = codestatus;
    }
    void setup(TaskHandle_t* t1, TaskHandle_t* t2, TaskHandle_t* t3, TaskHandle_t* t4, int sec = -1) {
        task1 = t1;  task2 = t2;  task3 = t3;  task4 = t4;
        if (sec >= 0) timeout_sec = sec;
        myprefs->begin("FlyByWire", false);
        bootcounter();
        set_codestatus(Booting);
        print_postmortem();
        recover_status();
        if (!watchdog_enabled) return;
        Serial.printf("Boot manager.. \n");
        esp_task_wdt_init(timeout_sec, true);  // see https://github.com/espressif/esp-idf/blob/master/examples/system/task_watchdog/main/task_watchdog_example_main.c
        esp_task_wdt_add(NULL);
    }
    void pet() {
        if (!watchdog_enabled) return;
        esp_task_wdt_reset();
    }
    void add(TaskHandle_t taskh) {
        if (!watchdog_enabled) return;
        esp_task_wdt_add(taskh);
    }
    void update() {
        pet();
        if (codestatus == Booting) set_codestatus(Confused);  // we are not booting any more
        write_uptime();
        print_high_water(task1, task2, task3, task4);
    }
  private:
    void bootcounter() {
        bootcount = myprefs->getUInt("bootcount", 0) + 1;
        myprefs->putUInt("bootcount", bootcount);
        codestatus_postmortem = myprefs->getUInt("codestatus", Confused);
        crashcount = myprefs->getUInt("crashcount", 0);
        if (codestatus_postmortem != Parked) crashcount++;
        myprefs->putUInt("crashcount", crashcount);
        was_panicked = (bool)myprefs->getUInt("panicstop", false);
    }
    void write_uptime() {
        float get_uptime = uptime_min;
        uint32_t myround = std::min((uint32_t)get_uptime, uptime_rounding);
        uint32_t uptime_new = (uint32_t)(get_uptime / (float)myround) * myround;
        if (uptime_new == uptime_recorded) return;
        myprefs->putUInt("uptime", uptime_new);
        uptime_recorded = uptime_new;
    }
    void print_postmortem() {
        Serial.printf("Boot count: %d (%d/%d). Last lost power while %s", bootcount, bootcount-crashcount, crashcount, codestatuscard[codestatus_postmortem].c_str());
        if (was_panicked) Serial.printf(" and panicking,");
        Serial.printf(" after ");
        uint32_t last_uptime = myprefs->getUInt("uptime", 0);
        if (last_uptime > 0) {
            Serial.printf("just over %d min uptime\n", last_uptime);
            write_uptime();
        }
        else Serial.printf("under 1 min uptime\n");
    }
    void print_high_water(xTaskHandle* t1, xTaskHandle* t2, xTaskHandle* t3, xTaskHandle* t4) {
        if (print_task_stack_usage && highWaterTimer.expireset()) {
            Serial.printf("mem minfree(B): heap:%d", xPortGetMinimumEverFreeHeapSize());            
            highWaterBytes = uxTaskGetStackHighWaterMark(*t1) * sizeof(StackType_t);
            Serial.printf(" temptask:%d", highWaterBytes);
            highWaterBytes = uxTaskGetStackHighWaterMark(*t2) * sizeof(StackType_t);
            Serial.printf(", webtask:%d", highWaterBytes);
            highWaterBytes = uxTaskGetStackHighWaterMark(*t3) * sizeof(StackType_t);
            Serial.printf(", drawtask:%d", highWaterBytes);
            highWaterBytes = uxTaskGetStackHighWaterMark(*t4) * sizeof(StackType_t);
            Serial.printf(", pushtask:%d\n", highWaterBytes);
        }
    }
    void recover_status() {
        if ((codestatus_postmortem != Driving && codestatus_postmortem != Stopped) || !crash_driving_recovery) return;
        if (was_panicked) {
            Serial.printf("  Continuing to panic..\n");
            ignition.panic_request(REQ_ON);
            return;
        }
        Serial.printf("  Resuming %s status..\n", codestatuscard[codestatus_postmortem]);
        boot_to_runmode = (codestatus_postmortem == Driving) ? FLY : HOLD;
        ignition.request(REQ_ON);
        // gas.(brake.pc[STOP]);  // brake.pid_targ_pc(brake.pc[STOP]);
    }
};
