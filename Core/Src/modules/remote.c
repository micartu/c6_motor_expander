#include "remote.h"
#ifdef USING_REAL_UART
#include "uart_comp.h"
#endif
#include "delay.h"
#include "crc8.h"
#include <string.h>
#include <assert.h>

static struct UART_Descr * _coms[MAX_CACHED_SZ];
static size_t _coms_count = 0;

#define MIN(a,b) (((a)<(b)) ? (a) : (b))

static void
check_command(struct UART_Descr * com);

#if USING_REAL_UART
static void 
start_over(struct UART_Descr * com)
{
	com->not_parsed_sz = 0;
	com->uart_ptr = 0;
	com->rstate = IDLE;
}

static void 
check_already_received(struct UART_Descr * com)
{
	if (com->not_parsed_sz > 0)
		check_command(com);
	switch (com->rstate)
	{
	case IDLE:
	case WFIRST_BYTE:
	case WLEN_BYTE:
		if (com->sz - com->uart_ptr < 4)
		{
			start_over(com);
		}
		break;
	case WREST_BYTES:
		if (com->sz <= com->uart_ptr ||
			com->pack_len > com->sz - com->uart_ptr)
		{
			// start afresh
			start_over(com);
		}
		break;

	default:
		break;
	}
}

/**
 * Changes the phase of the process of receiving
 * packet we're currently in
 * 
 * @param com describes UART we're working with
 */
static void 
change_rec_state(struct UART_Descr * com)
{
	check_already_received(com);
	switch (com->rstate)
	{
	case IDLE:
		com->uart_ptr = com->not_parsed_sz;
		com->rstate = WFIRST_BYTE;
		WRP_UART_Receive_IT(com->huart,
							com->uart_buffer + com->uart_ptr,
							1);
		com->rticks_elapsed = 0;
		break;

	case WFIRST_BYTE:
		if (com->uart_buffer[com->uart_ptr] == PACK_BEGIN)
		{
			com->rticks_elapsed = 0;
			com->uart_ptr = com->not_parsed_sz + 1;
			com->rstate = WLEN_BYTE;
			WRP_UART_Receive_IT(com->huart,
								com->uart_buffer + com->uart_ptr,
								1);
			break;
		} else {
			com->rstate = IDLE;
			// run it once again in order to receive next byte
			change_rec_state(com);
		}
		break;

	case WLEN_BYTE:
		com->uart_ptr = com->not_parsed_sz + 2;
		com->rstate = WREST_BYTES;
		com->pack_len = com->uart_buffer[com->uart_ptr - 1];
		if (com->sz <= com->uart_ptr ||
			com->pack_len > com->sz - com->uart_ptr)
		{
			// the packet is too big we cannot receive it,
			// let's wait another one, maybe buffer
			// would be freed by someone in run loop
			com->pack_len = 0;
			com->rstate = IDLE;
			change_rec_state(com);
		}
		else
		{
			com->rticks_elapsed = 0;
			WRP_UART_Receive_IT(com->huart,
								com->uart_buffer + com->uart_ptr,
								com->pack_len);
		}
		break;

	case WREST_BYTES:
		// data received, prepare next steps:
		com->not_parsed_sz = com->uart_ptr + com->pack_len;
		com->pack_len = 0;
		com->rticks_elapsed = 0;
		com->rstate = IDLE;
		// run it once again in order to wait for a next packet
		change_rec_state(com);
		break;
	}
}
#endif

static void
parse_data_and_call_void_fun(const uint8_t *buf,
							 uint8_t cmd,
							 size_t start_data,
							 size_t end_marker,
							 func_void_param_t callback)
{
	if (callback && end_marker - 1 >= start_data)
	{
		callback(cmd, buf + start_data, start_data - end_marker);
	}
}

static void
parse_data_and_call_uint32(const uint8_t *buf, size_t start_data,
						   size_t end_marker, func_uint32_paramed_t callback)
{
	const size_t sz = sizeof(uint32_t);
	if (callback && end_marker - 1 >= start_data + sz)
	{
		uint32_t data;
		memcpy(&data, buf + start_data, sz);
		callback(data);
	}
}

static void
cleanup_not_parsed_data(struct UART_Descr * com)
{
	const size_t len = (com->uart_ptr > com->not_parsed_sz) ?
						com->uart_ptr - com->not_parsed_sz : 0;
	if (len > 0) 
	{
		memmove(com->uart_buffer,
				com->uart_buffer + com->not_parsed_sz,
				len);
	}
	com->not_parsed_sz = 0;
	com->uart_ptr = len;
}

static void
check_command_in_buffer(struct UART_Descr * com, const uint8_t * buf, size_t sz)
{
	for (size_t i = 0; i < sz; )
	{
		if (buf[i] == PACK_BEGIN && i + 3 < sz)
		{
			const size_t len = buf[i + 1];
			if (i + len + 2 <= sz)
			{
				const size_t end_marker = i + len + 2 - 1;
				if (buf[end_marker] == PACK_END)
				{
					const size_t cmd_marker = i + 2;
					const uint8_t crc = crc8(buf + cmd_marker, end_marker - cmd_marker - 1);
					if (crc == buf[i + len])
					{
						// looks like a correct packet
						switch (buf[cmd_marker])
						{
						case CMD_TYPE_CUST:
							parse_data_and_call_uint32(buf, cmd_marker + 1,
													   end_marker, com->custom_cmd_p_ptr);
							break;

						case CMD_MOTOR_PWM:
							parse_data_and_call_void_fun(buf,
														 CMD_MOTOR_PWM,
														 cmd_marker + 1,
														 end_marker,
														 com->void_cmd_p_ptr);
							break;
						}
						i += len + 2;
						continue;
					}
				}
			}
		}
		++i;
	}
}

static void
check_command(struct UART_Descr * com)
{
	const uint8_t * buf = com->uart_buffer;
	check_command_in_buffer(com, buf, com->not_parsed_sz);
	cleanup_not_parsed_data(com);
}

// MARK: - init/deinit

void init_communication_uart(struct UART_Descr *com,
#ifdef USING_REAL_UART
							 UART_HandleTypeDef *huart,
#endif
							 uint8_t *buffer, size_t size)
{
	memset(com, 0, sizeof(*com));
#ifdef USING_REAL_UART
    com->huart = huart;
#endif
	com->uart_buffer = buffer;
	com->sz = size;
	assert(_coms_count < MAX_CACHED_SZ);
	_coms[_coms_count++] = com;
#ifdef USING_REAL_UART
	// initialize receiving data over the given port
	change_rec_state(com);
#endif
}

void deinit_communication_uart(const struct UART_Descr * com)
{
    for (size_t i = 0; i < _coms_count; ++i)
    {
        struct UART_Descr * com = _coms[i];
        if (com == _coms[i])
        {
			for (size_t j = i + 1; j <_coms_count; ++j)
			{
				_coms[j - 1] = _coms[j];
			}
			--_coms_count;
			break;
		}
	}
}

void install_custom_callback(struct UART_Descr * com, func_uint32_paramed_t func)
{
	com->custom_cmd_p_ptr = func;
}

// MARK: - events handling / parsing

#ifdef USING_REAL_UART
void uart_irq_handler(const UART_HandleTypeDef * huart)
{
	for (size_t i = 0; i < _coms_count; ++i)
	{
		struct UART_Descr * com = _coms[i];
		if (com->huart == huart)
		{
			// if we're here, then we've already received at least a byte,
			// so change state accordingly
			if (com->rstate == IDLE)
				com->rstate = WFIRST_BYTE;
			change_rec_state(com);
			break;
		}
	}
}
#endif

void uart_over_usb_data_received_handler(struct UART_Descr * com, uint8_t * buf, uint32_t len)
{
	// over the USB we receive all packets at once,
	// so try to parse them all at once
	com->not_parsed_sz = len;
	check_command_in_buffer(com, buf, com->not_parsed_sz);
}

void uart_systick_handler()
{
	for (size_t i = 0; i < _coms_count; ++i)
	{
		struct UART_Descr *com = _coms[i];
		if (com->rstate > WFIRST_BYTE)
			++(com->rticks_elapsed);
	}
}

void uart_runloop_handler()
{
	for (size_t i = 0; i < _coms_count; ++i)
	{
		struct UART_Descr * com = _coms[i];
		if (com->not_parsed_sz > 0)
		{
			check_command(com);
		}
		if (com->rticks_elapsed > SYS_TIMEOUT_TICKS)
		{
			com->rticks_elapsed = 0;
			com->uart_ptr = com->not_parsed_sz;
			com->rstate = IDLE;
		}
	}
}

#undef MIN