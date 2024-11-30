/*
 * SimpleParser.h
 *
 *  Created on: Aug 28, 2024
 *      Author: Szymon
 */

#ifndef INC_SIMPLEPARSER_H_
#define INC_SIMPLEPARSER_H_

#include "main.h"
#include "string.h"
#include "RingBuffer.h"
#include "Line_Follower.h"

#define ENDLINE '\n'
#define MY_NAME "GRUZIK2.0"

void Parser_TakeLine(RingBuffer_t *Buf, uint8_t *ReceivedData);
void Parser_Parse(uint8_t *ReceivedData, LineFollower_t *LineFollower);

#endif /* INC_SIMPLEPARSER_H_ */
