#ifndef USART_CH0_H_
#define USART_CH0_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

/* =========================================================
 * 자료형
 * ========================================================= */

typedef uint8_t  INT8;
typedef uint16_t INT16;

/* =========================================================
 * UART 송신 링 버퍼
 * ========================================================= */

/*
 * 38400bps에서 80ms마다 한 줄 정도 출력하는 용도라면
 * 128바이트 버퍼로 충분하다.
 */
#define USART0_TX_BUFFER_SIZE  128U

static volatile uint8_t usart0_tx_head = 0;
static volatile uint8_t usart0_tx_tail = 0;

static char usart0_tx_buffer[
    USART0_TX_BUFFER_SIZE
];

/*
 * 디버그 확인용
 *
 * 버퍼가 가득 차서 버려진 문자의 개수다.
 */
static volatile uint16_t usart0_tx_drop_count = 0;

/* =========================================================
 * 함수 선언
 * ========================================================= */

void USART0_Init(INT16 ubrr);

void USART0_Transmit(char data);

void USART0_Transmit_String(
    const char *str
);

void USART0_Transmit_NewLine(void);

char USART0_Receive(void);

uint8_t USART0_TxBufferEmpty(void);

uint16_t USART0_GetDropCount(void);

/* =========================================================
 * UART 초기화
 * ========================================================= */

void USART0_Init(INT16 ubrr)
{
    usart0_tx_head = 0;
    usart0_tx_tail = 0;
    usart0_tx_drop_count = 0;

    /*
     * U2X0 OFF
     *
     * main.c의 MYUBRR 계산은 반드시:
     *
     * F_CPU / (16 × BAUD) - 1
     */
    UCSR0A = 0x00;

    /*
     * Baud rate 설정
     */
    UBRR0H =
        (uint8_t)(ubrr >> 8);

    UBRR0L =
        (uint8_t)ubrr;

    /*
     * 수신, 송신 활성화
     *
     * UDR Empty 인터럽트는 여기서 켜지 않는다.
     * 송신할 문자가 버퍼에 들어오면 자동으로 켠다.
     */
    UCSR0B =
        (1 << RXEN0) |
        (1 << TXEN0);

    /*
     * 비동기 통신
     * 패리티 없음
     * Stop bit 1개
     * Data bit 8개
     */
    UCSR0C =
        (1 << UCSZ01) |
        (1 << UCSZ00);
}

/* =========================================================
 * UART 문자 송신 요청
 *
 * 실제 전송은 여기서 하지 않는다.
 * 문자를 링 버퍼에 저장하고 바로 반환한다.
 * ========================================================= */

void USART0_Transmit(char data)
{
    uint8_t next_head;

    next_head =
        (uint8_t)(
            usart0_tx_head + 1U
        );

    if (
        next_head >=
        USART0_TX_BUFFER_SIZE
    )
    {
        next_head = 0;
    }

    /*
     * 다음 head가 tail과 같다면
     * 송신 버퍼가 가득 찬 상태다.
     *
     * 제어 루프를 멈추지 않기 위해
     * 기다리지 않고 해당 문자를 버린다.
     */
    if (
        next_head ==
        usart0_tx_tail
    )
    {
        usart0_tx_drop_count++;
        return;
    }

    /*
     * 문자를 링 버퍼에 저장
     */
    usart0_tx_buffer[
        usart0_tx_head
    ] = data;

    /*
     * head 이동
     */
    usart0_tx_head =
        next_head;

    /*
     * UART Data Register Empty 인터럽트 활성화
     *
     * UART가 다음 문자를 받을 준비가 되면
     * USART0_UDRE_vect가 자동으로 실행된다.
     */
    UCSR0B |=
        (1 << UDRIE0);
}

/* =========================================================
 * 문자열 비동기 송신
 * ========================================================= */

void USART0_Transmit_String(
    const char *str
)
{
    while (*str != '\0')
    {
        USART0_Transmit(*str);
        str++;
    }
}

/* =========================================================
 * 줄바꿈 비동기 송신
 * ========================================================= */

void USART0_Transmit_NewLine(void)
{
    USART0_Transmit('\r');
    USART0_Transmit('\n');
}

/* =========================================================
 * UART Data Register Empty 인터럽트
 *
 * UART가 다음 문자를 받을 준비가 될 때마다 실행된다.
 * ========================================================= */

ISR(USART0_UDRE_vect)
{
    /*
     * head와 tail이 같다면
     * 보낼 문자가 없는 상태다.
     */
    if (
        usart0_tx_head ==
        usart0_tx_tail
    )
    {
        /*
         * 더 이상 인터럽트가 반복되지 않게
         * UDR Empty 인터럽트를 끈다.
         */
        UCSR0B &=
            ~(1 << UDRIE0);

        return;
    }

    /*
     * 링 버퍼의 다음 문자를
     * UART 송신 레지스터로 전달한다.
     */
    UDR0 =
        usart0_tx_buffer[
            usart0_tx_tail
        ];

    /*
     * tail 이동
     */
    usart0_tx_tail++;

    if (
        usart0_tx_tail >=
        USART0_TX_BUFFER_SIZE
    )
    {
        usart0_tx_tail = 0;
    }
}

/* =========================================================
 * 수신
 *
 * 수신은 기존과 동일한 blocking 방식이다.
 * 제어 루프 안에서 호출하면 제어가 멈출 수 있다.
 * ========================================================= */

char USART0_Receive(void)
{
    while (
        !(UCSR0A & (1 << RXC0))
    )
    {
        ;
    }

    return UDR0;
}

/* =========================================================
 * 송신 버퍼 상태
 * ========================================================= */

uint8_t USART0_TxBufferEmpty(void)
{
    if (
        usart0_tx_head ==
        usart0_tx_tail
    )
    {
        return 1;
    }

    return 0;
}

/* =========================================================
 * 버퍼 초과로 버린 문자 개수
 * ========================================================= */

uint16_t USART0_GetDropCount(void)
{
    uint8_t saved_sreg;
    uint16_t value;

    saved_sreg = SREG;

    cli();
    value = usart0_tx_drop_count;
    SREG = saved_sreg;

    return value;
}

#endif