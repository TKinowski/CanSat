#define CELL1_PIN A0
#define CELL2_PIN A1
#define CELL1_MULTIPLIER 3.3 / 1023
#define CELL2_MULTIPLIER 3.3 / 1023
double cell1_left = 0;
double cell2_left = 0;


void get_battery_left(){
    cell1_left = analogRead(CELL1_PIN) * CELL1_MULTIPLIER;
    cell2_left = analogRead(CELL2_PIN) * CELL2_MULTIPLIER;
}