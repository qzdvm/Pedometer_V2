#include "utils.h"



/**
 * \fn uint8_t o_rising_edge_detection(uint8_t)
 * \brief - state of 'catch' changes from 0 to 1
 * and return value remains 1 until which is read
 * \param catch - related value to catch
 * \return - 1 is caught
 */
bool o_rising_edge_detection(rising_edge_detection_t *s, bool o_catch)
{
	bool ret_val = false;
	if (o_catch)
	{
		if (!s->o_aux)
		{
			s->o_aux = true;
			ret_val = true;
		}
	}
	else
	{
		s->o_aux = false;
	}
	return ret_val;
}