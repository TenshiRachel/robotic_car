#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include <stdbool.h>
#include <string.h>

#include "components/wifi/car/wifi.h"

#define WHITE 0
#define BLACK 1
#define BARCODE_SIZE 9
#define GPIO_PIN 26
#define ADC_CHANNEL 0

typedef struct
{
    char character;
    bool success;
} characterValue;

typedef struct
{
    char character;
    char character_sequence[BARCODE_SIZE];
} characterSequence;


// void __not_in_flash_func(adc_capture)(uint16_t *buf, size_t count) {
//     adc_fifo_setup(true, false, 0, false, false);
//     adc_run(true);
//     for (int i = 0; i < count; i = i + 1)
//         buf[i] = adc_fifo_get_blocking();
//     adc_run(false);
//     adc_fifo_drain();
// }

characterSequence all_characters[] = {
    {'0', "000110100"},
    {'1', "100100001"},
    {'2', "001100001"},
    {'3', "101100000"},
    {'4', "000110001"},
    {'5', "100110000"},
    {'6', "001110000"},
    {'7', "000100101"},
    {'8', "100100100"},
    {'9', "001100100"},
    {'A', "100001001"}, 
    {'B', "001001001"},
    {'C', "101001000"},
    {'D', "000011001"},
    {'E', "100011000"},
    {'F', "001011000"}, 
    {'G', "000001101"},
    {'H', "100001100"},
    {'I', "001001100"},
    {'J', "000011100"},
    {'K', "100000011"},
    {'L', "001000011"},
    {'M', "101000010"},
    {'N', "000010011"},
    {'O', "100010010"},
    {'P', "001010010"},
    {'Q', "000000111"},
    {'R', "100000110"},
    {'S', "001000110"},
    {'T', "000010110"},
    {'U', "110000001"},
    {'V', "011000001"},
    {'W', "111000000"},
    {'X', "010010001"},
    {'Y', "110010000"},
    {'Z', "011010000"}
};

char asterisk[9] = {"010010100"};

void classify_timings(int8_t *timings_index, uint32_t *timings, char *classified_string);
characterValue check_character(char *classified_string, bool reverse_flag);
characterValue check_asterisk(char *classified_string, bool read_flag, bool end_flag, bool *reverse_flag);
int get_colour(uint32_t result);
bool process_barcode(__unused struct repeating_timer *t);

void ir_init_barcode(){
    adc_init();
    adc_gpio_init(GPIO_PIN); // set pin as the gpio adc input
    
    // Set pin 26 to input
    gpio_set_dir(GPIO_PIN, false);
    gpio_set_function(GPIO_PIN, GPIO_FUNC_SIO);
    adc_select_input(ADC_CHANNEL);

    // gpio_init(18);
    // gpio_set_dir(18, GPIO_OUT);
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
    static char classified_string[BARCODE_SIZE] = {0}; // binary string
    static int8_t num_existing_timings = 0;

    static int8_t colour = 2;
    static int8_t current_colour = 3;
    char message[48] = {0};

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

    static bool first_call = true;
    if (first_call) {
        startTime = get_absolute_time();
        first_call = false;
    }
    uint32_t result = adc_read();
    // printf("ADC: %u\n", result); // Use %u for uint32_t
    // const float converted_result = result * conversion_factor; // TODO: Optimise without multiplying
    current_colour = get_colour(result);
    // current_colour = gpio_get(27);
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

            // Print classified string
            // for (int i = 0; i < BARCODE_SIZE; i++) {
            //     printf("%c", classified_string[i]);
            // }
            // printf("\n");

            characterValue value;

            // If already read but hasn't ended, check for character
            if (!read_flag)
            {
                value = check_asterisk(classified_string, read_flag, end_flag, &reverse_flag);

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
                    char msg[] = "Found start * successfully!\n";
                    SendToMessageBuffer(msg, sizeof(msg), 0);
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
                    char msg[] = "Skip gap pulse!\n";
                    SendToMessageBuffer(msg, sizeof(msg), 0);
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
                        printf("Read a character %c! Now listening for end *.\n", character_read);

                        snprintf(message, sizeof(message), "Read a character %c! Now listening for end *.\n", character_read);
                        SendToMessageBuffer(message, sizeof(message), 0);
                    }
                    else // RESET EVERYTHING
                    {
                        char msg[] = "Invalid character. Resetting all\n";
                        SendToMessageBuffer(msg, sizeof(msg), 0);
                        printf("Invalid character. Resetting all\n");
                        read_flag = end_flag = gap_flag = reverse_flag = false;
                    }
                    num_existing_timings = 0;
                }

                // read end asterisk
                else
                {
                    value = check_asterisk(classified_string, read_flag, end_flag, &reverse_flag);
                    if (value.success)
                    {
                        snprintf(message, sizeof(message), "Successfully read character %c! Resetting to listen for start *.\n", character_read);
                        SendToMessageBuffer(message, sizeof(message), 0);
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

void classify_timings(int8_t *timings_index, uint32_t *timings, char *classified_string)
{
    // Calculate average of existing timings
    uint32_t sum = 0;
    for (int i = 0; i < BARCODE_SIZE; i++)
    {
        sum += timings[i];
    }

    uint32_t average = sum / BARCODE_SIZE;


    for (int i = 0; i < BARCODE_SIZE; i++)
    {
        int index = (*timings_index + i) % BARCODE_SIZE;
        if (timings[index] > average)
        {
            classified_string[i] = '1';
        }
        else
        {
            classified_string[i] = '0';
        }
    }
}

characterValue check_character(char *classified_string, bool reverse_flag)
{
    characterValue result;
    result.success = false;
    result.character = '\0';
    unsigned int barcode_size = BARCODE_SIZE;
    
    char temp_classified_string[BARCODE_SIZE];

    if (reverse_flag)
    {
        for (int i = 0; i < BARCODE_SIZE; i++)
        {
            temp_classified_string[i] = classified_string[BARCODE_SIZE - 1 - i];
        }
    }
    else
    {
        for (int i = 0; i < BARCODE_SIZE; i++)
        {
            temp_classified_string[i] = classified_string[i];
        }
    }

    // for each sequence of characters
    for (int i = 0; i < sizeof(all_characters) / sizeof(all_characters[0]); i++)
    {
        if (memcmp(temp_classified_string, all_characters[i].character_sequence, barcode_size) == 0)
        {
            result.success = true;
            result.character = all_characters[i].character;
            return result;
        }
    }
    return result;
}

characterValue check_asterisk(char *classified_string, bool read_flag, bool end_flag, bool *reverse_flag)
{
    characterValue result;
    result.success = false;
    result.character = '*';
    unsigned int barcode_size = BARCODE_SIZE;

    char temp_classified_string[BARCODE_SIZE];
    char temp_reversed_string[BARCODE_SIZE];

    for (int i = 0; i < BARCODE_SIZE; i++)
    {
        temp_classified_string[i] = classified_string[i];
    }

    for (int i = 0; i < BARCODE_SIZE; i++)
    {
        temp_reversed_string[i] = classified_string[BARCODE_SIZE - 1 - i];
    }

    // It is the first asterisk being read
    if (!read_flag)
    {
        // Check normal position
        if (memcmp(temp_classified_string, asterisk, barcode_size) == 0)
        {
            result.success = true;
        }

        // Check reversed position
        else if (memcmp(temp_reversed_string, asterisk, barcode_size) == 0)
        {
            result.success = true;
            *reverse_flag = true;
        }
    }

    // Reading end asterisk now, reverse direction should have been decided
    else if ((*reverse_flag && (memcmp(temp_reversed_string, asterisk, barcode_size) == 0)) ||
            (!*reverse_flag && (memcmp(temp_classified_string, asterisk, barcode_size) == 0)))
    {
        result.success = true;
    }


    return result;

    // // If it's the end, reverse is already decided
    // if (end_flag && *reverse_flag)
    // {
    //     for (int i = 0; i < 9; i++)
    //     {
    //         if (asterisk[i] != classified_string[9 - 1 - i])
    //         {
    //             result.success = false;
    //             break;
    //         }
    //     }
    //     return result;
    // }

    // // If it's the end and there is no reverse decided at the start
    // if (end_flag && !*reverse_flag)
    // {
    //     for (int i = 0; i < 9; i++)
    //     {
    //         if (asterisk[i] != classified_string[i])
    //         {
    //             result.success = false;
    //             break;
    //         }
    //     }
    //     return result;
    // }

    // // If it is not the end (reverse has not been decided), try both
    // // Try normal way first
    // for (int i = 0; i < 9; i++)
    // {
    //     if (asterisk[i] != classified_string[i])
    //     {
    //         result.success = false;
    //         break;
    //     }
    // }
    // // if successful, return result
    // if (result.success)
    // {
    //     return result;
    // }

    // // Otherwise try reverse version
    // result.success = true;
    // for (int i = 0; i < 9; i++)
    // {
    //     if (asterisk[i] != classified_string[9 - 1 - i])
    //     {
    //         // if not successful at any point, there is no match
    //         result.success = false;
    //         return result;
    //     }
    // }
    // // reached here, which means there is a successful match, hence set reverse flag
    // *reverse_flag = true;
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
