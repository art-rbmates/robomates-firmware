#ifndef SCAN_H
#define SCAN_H

class Scan {
public:
    static void update(bool r1_current_state, bool l1_current_state, bool &shouldRumble, bool inCrawlMode = false);
    static bool shouldSendScan();  // Check if scan should be sent (R1 pressed)
    
private:
    static bool scanBurstActive;
    static unsigned long scanBurstStartTime;
    static unsigned long scanCooldownEndTime;
    
    static bool r1ButtonLastState;
    static bool l1ButtonLastState;
    
    static bool shouldSendScanFlag;
    static unsigned long lastScanSentTime;

    static bool rumbleNeeded;
};

#endif // SCAN_H
