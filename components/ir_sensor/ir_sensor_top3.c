#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include <stdbool.h>

#define WHITE 0
#define BLACK 1
#define BARCODE_SIZE 9
#define THICK_COUNT 3
#define GPIO_PIN 27
#define ADC_CHANNEL 1

typedef struct
{
    char character;
    bool success;
} characterValue;

typedef struct
{
    char character;
    int character_sequence[THICK_COUNT];
} characterSequence;


characterSequence all_characters[] = {
    {'0', {3,4,6}},
    {'1', {0,3,8}}, 
    {'2', {2,3,8}},
    {'3', {0,2,3}},
    {'4', {3,4,8}},
    {'5', {0,3,4}},
    {'6', {2,3,4}}, 
    {'7', {3,6,8}},
    {'8', {0,3,6}}, 
    {'9', {2,3,6}},
    {'A', {0,5,8}}, 
    {'B', {2,5,8}},
    {'C', {0,2,5}},
    {'D', {4,5,8}},
    {'E', {0,4,5}},
    {'F', {2,4,5}},
    {'G', {5,6,8}},
    {'H', {0,5,6}},
    {'I', {2,5,6}},
    {'J', {4,5,6}},
    {'K', {0,7,8}},
    {'L', {2,7,8}},
    {'M', {0,2,7}},
    {'N', {4,7,8}},
    {'O', {0,4,7}},
    {'P', {2,4,7}},
    {'Q', {6,7,8}},
    {'R', {0,6,7}},
    {'S', {2,6,7}},
    {'T', {4,6,7}},
    {'U', {0,1,8}},
    {'V', {1,2,8}},
    {'W', {0,1,2}},
    {'X', {1,4,8}},
    {'Y', {0,1,4}},
    {'Z', {1,2,4}},
    {'-', {1,6,8}},
    {'.', {0,1,6}},
    {' ', {1,2,6}},
    {'$', {1,3,5}},
    {'/', {1,3,7}},
    {'+', {1,5,7}},
    {'%', {3,5,7}},
    {'*', {1,4,6}},
};

characterSequence asterisk = {'*', {1,4,6}};

void classify_timings(int8_t *timings_index, uint32_t *timings, int8_t *classified_string);
characterValue check_character(int8_t *classified_string, bool reverse_flag);
characterValue check_asterisk(int8_t *classified_string, bool end_flag, bool *reverse_flag);
int get_colour(uint32_t result);
bool process_barcode(__unused struct repeating_timer *t);

void ir_init_barcode(){
    adc_init();
    adc_gpio_init(GPIO_PIN); // set pin as the gpio adc input
    
    // Set pin 26 to input
    gpio_set_dir(GPIO_PIN, false);
    gpio_set_function(GPIO_PIN, GPIO_FUNC_SIO);
    adc_select_input(ADC_CHANNEL);

    gpio_init(18);
    gpio_set_dir(18, GPIO_OUT);
}
int state2 = 0;
bool process_barcode(struct repeating_timer *t)
{
    static bool read_flag = false;
    static bool end_flag = false;
    static bool gap_flag = false;
    static bool reverse_flag = false;
    static char character_read = '\0';

    static absolute_time_t startTime;
    static int8_t timings_index = 0;
    static uint32_t timings[BARCODE_SIZE] = {0}; // in milliseconds
    static int8_t classified_string[THICK_COUNT] = {0}; // array of size 3 showing the indexes
    static int8_t num_existing_timings = 0;

    static int8_t colour = 2;
    static int8_t current_colour = 3;

    static bool first_call = true;
    if (first_call) {
        startTime = get_absolute_time();
        first_call = false;
    }

    uint32_t result = adc_read();
    current_colour = get_colour(result);

    if (current_colour != colour) {
        // Calculate the pulse duration
        absolute_time_t endTime = get_absolute_time();
        uint64_t pulseDuration = absolute_time_diff_us(startTime, endTime);
        startTime = get_absolute_time();
        uint32_t timing_ms = pulseDuration / 1000;
        colour = current_colour;

        // Skip if too long or too short
        if (timing_ms > 3000 || timing_ms < 5)
        {
            return true;
        }

        // if (current_colour == WHITE) {
        //     printf("Black Duration: %u ms\n", timing_ms);
        // } else {
        //     printf("White Duration: %u ms\n", timing_ms);
        // }
        
        // Update timing in circular array
        timings[timings_index] = pulseDuration / 1000; 
        timings_index = (timings_index + 1) % BARCODE_SIZE; // Increment index in circular array
        if (num_existing_timings < 9)
        {
            num_existing_timings++;
        }

        // If there are already 9 characters
        if (num_existing_timings == BARCODE_SIZE)
        {
            // Classify the timings into a binary string of 9 values
            classify_timings(&timings_index, timings, classified_string);

            characterValue value;

            // If already read but hasn't ended, check for character
            if (!read_flag)
            {
                value = check_asterisk(classified_string, end_flag, &reverse_flag);

                // if(state2 == 0)
                // {
                //     state2 = 1;
                //     gpio_put(18,1);
                // }
                // else
                // {
                //     state2 = 0;
                //     gpio_put(18,0);
                // }

                // if successful, move on to read the actual character
                if (value.success)
                {
                    read_flag = true; // set flag to read the character
                    gap_flag = true; // set flag to skip the gap pulse
                    printf("Found start * successfully!\n");

                    // reset array to no timings
                    num_existing_timings = 0;
                }
                // if not successful, let the circular array continue searching for asterisk
            }

            // Else if initial * has been read, it's either reading character or end *
            else
            {
                // skip gap pulse before or after a character
                if (gap_flag)
                {
                    gap_flag = false;
                    printf("Skip gap pulse!\n");
                    return true;
                }
                
                // If end flag not set, read character
                if (!end_flag)
                {
                    // Read the character
                    value = check_character(classified_string, reverse_flag);
                    
                    // If successful in reading, set flags to read end
                    if (value.success)
                    {
                        end_flag = true; // signal to go detect for end character
                        gap_flag = true; // set flag to skip the gap pulse
                        character_read = value.character;
                        printf("Read a character! Now listening for end *.\n");
                    }
                    else // RESET EVERYTHING
                    {
                        printf("Invalid character. Resetting all\n");
                        read_flag = end_flag = gap_flag = reverse_flag = false;
                    }
                    num_existing_timings = 0;
                }

                // read end asterisk
                else
                {
                    value = check_asterisk(classified_string, end_flag, &reverse_flag);
                    if (value.success)
                    {
                        printf("Successfully read character %c! Resetting to listen for start *.\n", character_read);
                    }
                    else
                {
                    printf("Failed reading end asterisk. Resetting everything.\n");
                }
                    // Whether success/fail, reset flags and continue listening for start asterisk
                    read_flag = end_flag = gap_flag = reverse_flag = false;
                }

            }
        }
    }
    return true;
}

void classify_timings(int8_t *timings_index, uint32_t *timings, int8_t *classified_string)
{
    uint32_t top_timings[3] = {0};
    int8_t top_indexes[3] = {-1, -1, -1};

    for (int i = 0; i < BARCODE_SIZE; i++)
    {
        for (int j = 0; j < 3; j++) {
            // if current timing is bigger than one of the top timings
            if (timings[i] > top_timings[j])
            {
                // move timings backwards
                for (int k = 2; k > j; k--)
                {
                    top_timings[k] = top_timings[k - 1];
                    top_indexes[k] = top_indexes[k - 1];
                }
                // slot in current timing in this position
                top_timings[j] = timings[i];
                top_indexes[j] = timings_index[i];
                break;
            }
        }
    }


    // sort indexes in numerical order
    for (int i = 0; i < 2; i++) {
        for (int j = i + 1; j < 3; j++) {
            if (top_indexes[i] > top_indexes[j]) {
                int8_t temp_index = top_indexes[i];
                top_indexes[i] = top_indexes[j];
                top_indexes[j] = temp_index;
            }
        }
    }

    // insert into classified string
    for (int i = 0; i < 3; i++) {
        classified_string[i] = top_indexes[i];
    }
}

characterValue check_character(int8_t *classified_string, bool reverse_flag)
{
    characterValue result;
    result.success = false;
    result.character = '\0';

    // for each sequence of characters
    for (int i = 0; i < 3; i++)
    {
        // assume it's a match first
        result.success = true;
        result.character = all_characters[i].character;


        if (!reverse_flag)
        {
            // check thick line match
            for (int j = 0; j < 9; j++)
            {
                // break to go to next sequence if it is not a match
                if (all_characters[i].character_sequence[j] != classified_string[j])
                {
                    result.success = false;
                    break;
                }
            }
        }
        else
        {
            // check every character
            for (int j = 0; j < 9; j++)
            {
                // break to go to next sequence if it is not a match
                if (all_characters[i].character_sequence[j] != classified_string[9 - 1 - j])
                {
                    result.success = false;
                    break;
                }
            } 
        }
        
        // if it is still a success after all characters, this is A MATCH!
        if (result.success)
        {
            return result;
        }
    }
    return result;
}

characterValue check_asterisk(int8_t *classified_string, bool end_flag, bool *reverse_flag)
{
    characterValue result;
    result.success = true;
    result.character = '*';

    // If it's the end, reverse is already decided
    if (end_flag && *reverse_flag)
    {
        for (int i = 0; i < 9; i++)
        {
            if (asterisk[i] != classified_string[9 - 1 - i])
            {
                result.success = false;
                break;
            }
        }
        return result;
    }

    // If it's the end and there is no reverse decided at the start
    if (end_flag && !*reverse_flag)
    {
        for (int i = 0; i < 9; i++)
        {
            if (asterisk[i] != classified_string[i])
            {
                result.success = false;
                break;
            }
        }
        return result;
    }

    // If it is not the end (reverse has not been decided), try both
    // Try normal way first
    for (int i = 0; i < 9; i++)
    {
        if (asterisk[i] != classified_string[i])
        {
            result.success = false;
            break;
        }
    }
    // if successful, return result
    if (result.success)
    {
        return result;
    }

    // Otherwise try reverse version
    result.success = true;
    for (int i = 0; i < 9; i++)
    {
        if (asterisk[i] != classified_string[9 - 1 - i])
        {
            // if not successful at any point, there is no match
            result.success = false;
            return result;
        }
    }
    // reached here, which means there is a successful match, hence set reverse flag
    *reverse_flag = true;
    return result;
}

int get_colour(uint32_t result)
{
    static uint32_t min_adc = 4095;
    static uint32_t max_adc = 0;
    static uint32_t contrast_adc = 4095;

    if (result < min_adc)
    {
        min_adc = result;
        contrast_adc = (uint32_t)((min_adc + max_adc)/2);
    }
    else if (result > max_adc)
    {
        max_adc = result;
        contrast_adc = (uint32_t)((min_adc + max_adc)/2);
    }
    if (result >= contrast_adc)
    {
        return BLACK;
    }
    else
    {
        return WHITE;
    }
}

// void clear_timings()
// {
//     for (int i = 0; i < 9; i++)
//     {
//         timings[i] = 0;
//         num_existing_timings = 0;
//         classified_string[i] = 0;
//     }
// }

// #include <stdio.h>
// #include "pico/stdlib.h"
// #include "hardware/gpio.h"
// #include "hardware/adc.h"
// #include "hardware/irq.h"
// #include "hardware/timer.h"

// volatile int colour = 2;
// volatile uint32_t result = 0;

// // repeating timer
// bool adc_timer_callback(struct repeating_timer *t) {
//     result = adc_read();
    
//     const float conversion_factor = 3.3f / (1 << 12);
//     const float converted_result = result * conversion_factor;

//     if (converted_result > 0.6) {
//         if (colour != 1) {
//             printf("Black surface detected\n");
//             colour = 1;
//         }
//     } else {
//         if (colour != 0) {
//             printf("White surface detected\n");
//             colour = 0;
//         }
//     }
//     return true;  // returning true = the timer will repeat
// }

// int main(void) {
//     stdio_init_all();
//     adc_init();
//     adc_gpio_init(26);
//     adc_select_input(0);

//     struct repeating_timer timer;

//     add_repeating_timer_us(-100000, adc_timer_callback, NULL, &timer);

//     // Main loop does nothing, as the work is handled by the interrupt-like timer
//     while (1) {
//         tight_loop_contents();  // Wait for the timer to trigger
//     }
// }
