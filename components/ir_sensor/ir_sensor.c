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
#define FULL_BARCODE_SIZE 29
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
    {'Z', "011010000"},
};

char asterisk[9] = {"010010100"};
static uint32_t running_total = 0;

void classify_timings(int8_t *timings_index, uint32_t *timings, char *classified_string);
characterValue check_character(char *classified_string, bool reverse_flag);
characterValue check_asterisk(char *classified_string, bool read_flag, bool *reverse_flag);
int get_colour(uint32_t result);
bool process_barcode(__unused struct repeating_timer *t);

void ir_init_barcode()
{
    adc_init();
    adc_gpio_init(GPIO_PIN); // set pin as the gpio adc input
    
    // Set pin 26 to input
    gpio_set_dir(GPIO_PIN, false);
    gpio_set_function(GPIO_PIN, GPIO_FUNC_SIO);
    adc_select_input(ADC_CHANNEL);

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

    static uint32_t timings_full[FULL_BARCODE_SIZE] = {0}; // in milliseconds
    static char classified_string_full[FULL_BARCODE_SIZE] = {0}; // binary string

    static int8_t num_existing_timings = 0;

    static int8_t colour = 2;
    static int8_t current_colour = 3;
    char message[64] = {0};

    static bool first_call = true;
    if (first_call)
    {
        startTime = get_absolute_time();
        first_call = false;
    }
    uint32_t result = adc_read();
    // printf("ADC: %u\n", result); // Use %u for uint32_t
    current_colour = get_colour(result);
    // current_colour = gpio_get(27);
    if (current_colour != colour)
    {
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
        
        // If haven't filled timings 
        if (num_existing_timings < FULL_BARCODE_SIZE)
        {
            num_existing_timings++;
            running_total += timing_ms;
        }
        else // do moving average
        {
            // Remove previous timing and add new timing to total
            running_total = running_total - timings_full[timings_index] + timing_ms;
        }

        // Replace timing in index
        timings_full[timings_index] = timing_ms;
        timings_index = (timings_index + 1) % FULL_BARCODE_SIZE; // Increment index in circular array

        // If there are already 27 characters
        if (num_existing_timings == FULL_BARCODE_SIZE)
        {
            // Classify the timings into a binary string of 29 values
            classify_timings(&timings_index, timings_full, classified_string_full);

            characterValue value;

            // for (int i = 0; i < FULL_BARCODE_SIZE; i++)
            // {
            //     printf("%c", classified_string_full[i]);
            // }
            // printf("\n");

            // If the 'gap' values are not thin lines
            if (!(classified_string_full[9] == '0') || !(classified_string_full[19] == '0'))
            {
                // printf("Gap values not thin lines\n");
                return true;
            }

            // Check asterisk first
            value = check_asterisk(classified_string_full, read_flag, &reverse_flag);
            if (!value.success)
            {
                // printf("First asterisk fail\n");
                return true;
            }

            // Check for character
            value = check_character(&classified_string_full[10], reverse_flag);
            if (!value.success)
            {
                // printf("Character check fail\n");
                reverse_flag = false;
                return true;
            }
            character_read = value.character;
            read_flag = true;

            // Check for end asterisk
            value = check_asterisk(&classified_string_full[20], read_flag, &reverse_flag);
            if (!value.success)
            {
                // printf("End asterisk fail\n");
                read_flag = false;
                reverse_flag = false;
                return true;
            }

            printf("Successfully read character %c\n", character_read);
            read_flag = false;
            reverse_flag = false;
            return true;
        }
    }
    return true;
}

void classify_timings(int8_t *timings_index, uint32_t *timings, char *classified_string)
{
    uint32_t average = running_total / FULL_BARCODE_SIZE;
    for (int i = 0; i < FULL_BARCODE_SIZE; i++)
    {
        int index = (*timings_index + i) % FULL_BARCODE_SIZE;
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

characterValue check_asterisk(char *classified_string, bool read_flag, bool *reverse_flag)
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