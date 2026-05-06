#include "pinnames.h"


#if 0
void foo_char_ptr(int a, char *b) {

	for (int i=0;i<2;i++) a++;
	
	if (a==0) return;
}

void foo_int(int a, int b) {
	for (int i=0;i<a;i++) b++;
	
	if (a==0) return;
}

#define foo(a, b) _Generic((b), char*: foo_char_ptr, int: foo_int)(a, b)
#endif

uint32_t pinmap_merge(uint32_t a, uint32_t b)
{
    // both are the same (inc both NC)
    if (a == b) {
        return a;
    }

    // one (or both) is not connected
    if (a == (uint32_t)NC) {
        return b;
    }
    if (b == (uint32_t)NC) {
        return a;
    }

    // mis-match error case
    //pinmap mis-match
		return a;
}

uint32_t pinmap_find_peripheral(PinName pin, const PinMap *map)
{
    while (map->pin != NC) {
        if (map->pin == pin) {
            return map->peripheral;
        }
        map++;
    }
    return (uint32_t)NC;
}

uint32_t pinmap_peripheral(PinName pin, const PinMap *map)
{
    uint32_t peripheral = (uint32_t)NC;

    if (pin == (PinName)NC) {
        return (uint32_t)NC;
    }
    peripheral = pinmap_find_peripheral(pin, map);
    if ((uint32_t)NC == peripheral) { // no mapping available
        //pinmap not found for peripheral
    }
    return peripheral;
}

uint32_t pinmap_find_function(PinName pin, const PinMap *map)
{
    while (map->pin != NC) {
        if (map->pin == pin) {
            return map->function;
        }
        map++;
    }
    return (uint32_t)NC;
}

uint32_t pinmap_function(PinName pin, const PinMap *map)
{
    uint32_t function = (uint32_t)NC;

    if (pin == (PinName)NC) {
        return (uint32_t)NC;
    }
    function = pinmap_find_function(pin, map);
    if ((uint32_t)NC == function) { // no mapping available
        //pinmap not found for function
    }
    return function;
}

uint32_t pinmap_find_channel(PinName pin, uint32_t peripheral, const PinMap *map)
{
    while (map->pin != NC) {
        if ((map->pin == pin) && (map->peripheral == peripheral)) {
            return map->channel;
        }
        map++;
    }
    return (uint32_t)NC;
}

uint32_t pinmap_channel(PinName pin, uint32_t peripheral, const PinMap *map)
{
    uint32_t channel = (uint32_t)NC;

    if (pin == (PinName)NC) {
        return (uint32_t)NC;
    }
    channel = pinmap_find_channel(pin, peripheral, map);
    if ((uint32_t)NC == channel) { // no mapping available
        //pinmap not found for function
    }
    return channel;
}


uint32_t pinmap_find_function_ex(PinName pin, uint32_t peripheral, const PinMap *map)
{
    while (map->pin != NC) {
        if ((map->pin == pin) && (map->peripheral == peripheral)) {
            return map->function;
        }
        map++;
    }
    return (uint32_t)NC;
}

uint32_t pinmap_function_ex(PinName pin, uint32_t peripheral, const PinMap *map)
{
    uint32_t function = (uint32_t)NC;

    if (pin == (PinName)NC) {
        return (uint32_t)NC;
    }
    function = pinmap_find_function_ex(pin, peripheral, map);
    if ((uint32_t)NC == function) { // no mapping available
        //pinmap not found for function
    }
    return function;
}
