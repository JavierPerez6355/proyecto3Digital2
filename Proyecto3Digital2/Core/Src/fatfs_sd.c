/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fatfs_sd.c
  * @brief   Controlador de tarjeta microSD por SPI1 para FatFs.
  * @note    Adaptado al proyecto STM32F446RE con pantalla ILI9341.
  *          Version de diagnostico: verificar resultados antes de usar
  *          para almacenar mediciones definitivas.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define TRUE  1
#define FALSE 0
#define bool BYTE
#include "stm32f4xx_hal.h"
#include "diskio.h"
#include "fatfs_sd.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Definiciones privadas -----------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Variables privadas --------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern void SPI1_VelocidadBaja(void);
extern void SPI1_VelocidadAlta(void);
extern UART_HandleTypeDef huart2;
// Contadores actualizados por SysTick.
volatile uint16_t Timer1, Timer2;           /* Contadores de 1 ms */
static volatile DSTATUS Stat = STA_NOINIT;  /* Estado del disco */
static uint8_t CardType;                    /* Tipo de tarjeta y modo de direccionamiento */
static uint8_t PowerFlag = 0;               /* Indicador de alimentacion */
/* USER CODE END PV */

/* Funciones privadas --------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/***************************************
 * Funciones de comunicacion SPI
 **************************************/
/* Seleccionar tarjeta */
static void SELECT(void)
{
    // Deseleccionar pantalla ILI9341
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
    // Seleccionar microSD
    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
}
/* Deseleccionar tarjeta */
static void DESELECT(void)
{
    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET);
    HAL_Delay(1);
}
/* El estado de SPI se comprueba al finalizar las operaciones de disco. */
static volatile HAL_StatusTypeDef SD_SPI_Error = HAL_OK;
static volatile uint8_t SD_DebugResponse = 0xFF;
static volatile uint8_t SD_DebugFinal = 0xFF;
static volatile uint8_t SD_DebugResult = 0;

static void SPI_TxByte(uint8_t data)
{
    HAL_StatusTypeDef r = HAL_SPI_Transmit(HSPI_SDCARD, &data, 1, SPI_TIMEOUT);
    if (r != HAL_OK) SD_SPI_Error = r;
}
static void SPI_TxBuffer(uint8_t *buffer, uint16_t len)
{
    HAL_StatusTypeDef r = HAL_SPI_Transmit(HSPI_SDCARD, buffer, len, SPI_TIMEOUT);
    if (r != HAL_OK) SD_SPI_Error = r;
}
static uint8_t SPI_RxByte(void)
{
    uint8_t dummy = 0xFF, data = 0xFF;
    HAL_StatusTypeDef r = HAL_SPI_TransmitReceive(HSPI_SDCARD, &dummy, &data, 1, SPI_TIMEOUT);
    if (r != HAL_OK) SD_SPI_Error = r;
    return data;
}
/* Recibir un byte mediante puntero */
static void SPI_RxBytePtr(uint8_t *buff) 
{
    *buff = SPI_RxByte();
}
/***************************************
 * Funciones de la tarjeta SD
 **************************************/
/* Esperar a que la tarjeta este lista */
static uint8_t SD_ReadyWait(void)
{
    uint8_t res;
    /* Tiempo maximo de espera: 500 ms */
    Timer2 = 500;
    /* La tarjeta responde 0xFF cuando esta lista */
    do {
        res = SPI_RxByte();
    } while ((res != 0xFF) && Timer2);
    return res;
}
/* Activacion logica de tarjeta */
static void SD_PowerOn(void) 
{
    uint8_t args[6];
    uint32_t cnt = 0x1FFF;
    /* Enviar pulsos de reloj para activar la tarjeta */
    DESELECT();
    for(int i = 0; i < 10; i++)
    {
        SPI_TxByte(0xFF);
    }
    /* Seleccionar tarjeta */
    SELECT();
    /* Solicitar estado de reposo */
    args[0] = CMD0;     /* CMD0:GO_IDLE_STATE */
    args[1] = 0;
    args[2] = 0;
    args[3] = 0;
    args[4] = 0;
    args[5] = 0x95;     /* CRC */
    SPI_TxBuffer(args, sizeof(args));
    /* Esperar respuesta */
    while ((SPI_RxByte() != 0x01) && cnt)
    {
        cnt--;
    }
    DESELECT();
    SPI_TxByte(0XFF);
    PowerFlag = 1;
}
/* Desactivacion logica de tarjeta */
static void SD_PowerOff(void) 
{
    PowerFlag = 0;
}
/* Consultar indicador de alimentacion */
static uint8_t SD_CheckPower(void) 
{
    return PowerFlag;
}
/* Recibir bloque de datos */
static bool SD_RxDataBlock(BYTE *buff, UINT len)
{
    uint8_t token;
    /* Tiempo maximo de espera: 200 ms */
    Timer1 = 200;
    /* Esperar respuesta o vencimiento del tiempo */
    do {
        token = SPI_RxByte();
    } while((token == 0xFF) && Timer1);
    /* Respuesta invalida */
    if(token != 0xFE) return FALSE;
    /* Recibir datos */
    // Leer exactamente len bytes.
    while(len--) {
        SPI_RxBytePtr(buff++);
    }
    // Descartar los dos bytes CRC recibidos
    SPI_RxByte();
    SPI_RxByte();
    return TRUE;
}
/* Transmitir bloque de datos */
#if _USE_WRITE == 1
static bool SD_TxDataBlock(const uint8_t *buff, BYTE token)
{
    uint8_t resp = 0xFF;
    uint8_t estado = 0x00;
    uint8_t i;

    if (SD_ReadyWait() != 0xFF || SD_SPI_Error != HAL_OK) return FALSE;
    if (token != 0xFD && buff == NULL) return FALSE;
    SPI_TxByte(token);
    if (SD_SPI_Error != HAL_OK) return FALSE;
    if (token == 0xFD) {
        estado = SD_ReadyWait();
        return (estado == 0xFF && SD_SPI_Error == HAL_OK);
    }
    SPI_TxBuffer((uint8_t *)buff, 512);
    SPI_TxByte(0xFF);
    SPI_TxByte(0xFF);
    if (SD_SPI_Error != HAL_OK) return FALSE;

    for (i = 0; i < 65; ++i) {
        resp = SPI_RxByte();
        if (SD_SPI_Error != HAL_OK) return FALSE;
        if ((resp & 0x1F) != 0x1F) break;
    }
    if ((resp & 0x1F) != 0x05) {
        SD_DebugResponse = resp;
        SD_DebugFinal = 0xFF;
        SD_DebugResult = 1;
        return FALSE;
    }
    Timer1 = 500;
    do {
        estado = SPI_RxByte();
        if (SD_SPI_Error != HAL_OK) break;
    } while (estado != 0xFF && Timer1 > 0);

    SD_DebugResponse = resp;
    SD_DebugFinal = estado;
    SD_DebugResult = (estado == 0xFF && SD_SPI_Error == HAL_OK) ? 0 : 2;
    return SD_DebugResult == 0;
}
#endif /* _USE_WRITE */
/* Transmitir comando */
static BYTE SD_SendCmd(BYTE cmd, uint32_t arg)
{
    uint8_t crc, res;
    /* Esperar a que la tarjeta este lista */
    if (SD_ReadyWait() != 0xFF) return 0xFF;
    /* Transmitir comando */
    SPI_TxByte(cmd);                    /* Comando */
    SPI_TxByte((uint8_t)(arg >> 24));   /* Argumento[31..24] */
    SPI_TxByte((uint8_t)(arg >> 16));   /* Argumento[23..16] */
    SPI_TxByte((uint8_t)(arg >> 8));    /* Argumento[15..8] */
    SPI_TxByte((uint8_t)arg);           /* Argumento[7..0] */
    /* Preparar CRC */
    if(cmd == CMD0) crc = 0x95; /* CRC para CMD0(0) */
    else if(cmd == CMD8) crc = 0x87;    /* CRC para CMD8(0x1AA) */
    else crc = 1;
    /* Transmitir CRC */
    SPI_TxByte(crc);
    /* Descartar byte adicional despues de CMD12 */
    if (cmd == CMD12) SPI_RxByte();
    /* Recibir respuesta */
    uint8_t n = 10;
    do {
        res = SPI_RxByte();
    } while ((res & 0x80) && --n);
    return res;
}
/***************************************
 * Funciones de interfaz con FatFs
 **************************************/
/* Inicializar tarjeta SD */
DSTATUS SD_disk_initialize(BYTE drv) 
{
    uint8_t n, type, ocr[4];
    /* Solo se admite la unidad fisica cero */
    if(drv) return STA_NOINIT;
    /* Tarjeta no disponible */
    if(Stat & STA_NODISK) return Stat;
    // Configurar SPI1 a baja velocidad
    SD_SPI_Error = HAL_OK;
   SPI1_VelocidadBaja();
    /* Activacion logica de tarjeta */
    SD_PowerOn();
    /* Seleccionar tarjeta */
    SELECT();
    /* Identificar tipo de tarjeta */
    type = 0;
    /* Enviar comando de reposo CMD0 */
    if (SD_SendCmd(CMD0, 0) == 1)
    {
        /* Tiempo maximo: 1 segundo */
        Timer1 = 1000;
        /* SD version 2+ accept CMD8 command, http://elm-chan.org/docs/mmc/mmc_e.html */
        if (SD_SendCmd(CMD8, 0x1AA) == 1)
        {
            /* Leer registro de condiciones de operacion */
            for (n = 0; n < 4; n++)
            {
                ocr[n] = SPI_RxByte();
            }
            /* Rango de tension de 2.7 a 3.6 V */
            if (ocr[2] == 0x01 && ocr[3] == 0xAA)
            {
                /* Enviar ACMD41 con indicador HCS */
                do {
                    if (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(CMD41, 1UL << 30) == 0) break;
                } while (Timer1);
                /* READ_OCR */
                if (Timer1 && SD_SendCmd(CMD58, 0) == 0)
                {
                    /* Comprobar bit CCS */
                    for (n = 0; n < 4; n++)
                    {
                        ocr[n] = SPI_RxByte();
                    }
                    /* Tarjeta SD version 2 (alta o estandar capacidad) */
                    type = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;
                }
            }
        }
        else
        {
            /* Tarjeta SD version 1 o MMC */
            type = (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(CMD41, 0) <= 1) ? CT_SD1 : CT_MMC;
            do
            {
                if (type == CT_SD1)
                {
                    if (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(CMD41, 0) == 0) break; /* ACMD41 */
                }
                else
                {
                    if (SD_SendCmd(CMD1, 0) == 0) break; /* CMD1 */
                }
            } while (Timer1);
            /* Configurar longitud de bloque */
            if (!Timer1 || SD_SendCmd(CMD16, 512) != 0) type = 0;
        }
    }
    CardType = type;
    /* Dejar bus en reposo */
    DESELECT();
    SPI_RxByte();
    /* Limpiar indicador de tarjeta no inicializada */
    if (type)
    {
    	// Inicializacion exitosa
    	Stat &= ~STA_NOINIT;
    	// Cambiar SPI1 a velocidad de transferencia
    	SPI1_VelocidadAlta();
    }
    else
    {
        /* Fallo de inicializacion */
    	// Inicializacion fallida
    	    SD_PowerOff();
    	    // Restaurar SPI1 para la pantalla
    	    SPI1_VelocidadAlta();
    }
    return Stat;
}
/* Consultar estado de disco */
DSTATUS SD_disk_status(BYTE drv) 
{
    if (drv) return STA_NOINIT;
    return Stat;
}
/* Leer sectores */
DRESULT SD_disk_read(BYTE pdrv, BYTE* buff, DWORD sector, UINT count) 
{
    /* La unidad fisica debe ser cero */
    if (pdrv || !count) return RES_PARERR;
    /* Tarjeta no disponible */
    if (Stat & STA_NOINIT) return RES_NOTRDY;
    /* Convertir sector a direccion en bytes */
    if (!(CardType & CT_BLOCK)) sector *= 512;
    SD_SPI_Error = HAL_OK;
   SELECT();
    if (count == 1)
    {
        /* READ_SINGLE_BLOCK */
        if ((SD_SendCmd(CMD17, sector) == 0) && SD_RxDataBlock(buff, 512)) count = 0;
    }
    else
    {
        /* READ_MULTIPLE_BLOCK */
        if (SD_SendCmd(CMD18, sector) == 0)
        {
            do {
                if (!SD_RxDataBlock(buff, 512)) break;
                buff += 512;
            } while (--count);
            /* Detener transmision */
            SD_SendCmd(CMD12, 0);
        }
    }
    /* Dejar bus en reposo */
    DESELECT();
    SPI_RxByte();
    return (count || SD_SPI_Error != HAL_OK) ? RES_ERROR : RES_OK;
}
/* Escribir sectores */
#if _USE_WRITE == 1
DRESULT SD_disk_write(BYTE pdrv, const BYTE* buff, DWORD sector, UINT count) 
{
    /* La unidad fisica debe ser cero */
    if (pdrv || !count) return RES_PARERR;
    /* Tarjeta no disponible */
    if (Stat & STA_NOINIT) return RES_NOTRDY;
    /* Proteccion contra escritura */
    if (Stat & STA_PROTECT) return RES_WRPRT;
    /* Convertir sector a direccion en bytes */
    if (!(CardType & CT_BLOCK)) sector *= 512;
    SD_SPI_Error = HAL_OK;
   SELECT();
    if (count == 1)
    {
        /* WRITE_BLOCK */
        if ((SD_SendCmd(CMD24, sector) == 0) && SD_TxDataBlock(buff, 0xFE))
            count = 0;
    }
    else
    {
        /* WRITE_MULTIPLE_BLOCK */
        if (CardType & CT_SD1)
        {
            SD_SendCmd(CMD55, 0);
            SD_SendCmd(CMD23, count); /* ACMD23 */
        }
        if (SD_SendCmd(CMD25, sector) == 0)
        {
            do {
                if(!SD_TxDataBlock(buff, 0xFC)) break;
                buff += 512;
            } while (--count);
            /* Token de fin de escritura multiple */
            if(!SD_TxDataBlock(0, 0xFD))
            {
                count = 1;
            }
        }
    }
    /* Dejar bus en reposo */
    DESELECT();
    SPI_RxByte();
    return (count || SD_SPI_Error != HAL_OK) ? RES_ERROR : RES_OK;
}
#endif /* _USE_WRITE */
/* ioctl */
DRESULT SD_disk_ioctl(BYTE drv, BYTE ctrl, void *buff) 
{
    DRESULT res;
    uint8_t n, csd[16], *ptr = buff;
    /* La unidad fisica debe ser cero */
    if (drv) return RES_PARERR;
    res = RES_ERROR;
   SD_SPI_Error = HAL_OK;
    if (ctrl == CTRL_POWER)
    {
        switch (*ptr)
        {
        case 0:
            SD_PowerOff();      /* Desactivar */
            res = RES_OK;
            break;
        case 1:
            SD_PowerOn();       /* Activar */
            res = RES_OK;
            break;
        case 2:
            *(ptr + 1) = SD_CheckPower();
            res = RES_OK;       /* Consultar estado */
            break;
        default:
            res = RES_PARERR;
        }
    }
    else
    {
        /* Tarjeta no disponible */
        if (Stat & STA_NOINIT) return RES_NOTRDY;
        SELECT();
        switch (ctrl)
        {
        case GET_SECTOR_COUNT:
            /* SEND_CSD */
            if ((SD_SendCmd(CMD9, 0) == 0) && SD_RxDataBlock(csd, 16))
            {
                if ((csd[0] >> 6) == 1) /* SD version 2 */
                {
                    // Correccion del calculo de capacidad SDv2
                    // Se utiliza el campo C_SIZE completo para calcular la capacidad.
                    DWORD c_size;
                    c_size = (DWORD)(csd[7] & 0x3F) << 16 | (WORD)csd[8] << 8 | csd[9];
                    *(DWORD*)buff = (c_size + 1) << 10;
                }
                else /* MMC o SD version 1 */
                {
                    WORD csize;
                    n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
                    csize = (csd[8] >> 6) + ((WORD) csd[7] << 2) + ((WORD) (csd[6] & 3) << 10) + 1;
                    *(DWORD*) buff = (DWORD) csize << (n - 9);
                }
                res = RES_OK;
            }
            break;
        case GET_SECTOR_SIZE:
            *(WORD*) buff = 512;
            res = RES_OK;
            break;
        case CTRL_SYNC:
            if (SD_ReadyWait() == 0xFF) res = RES_OK;
            break;
        case MMC_GET_CSD:
            /* SEND_CSD */
            if (SD_SendCmd(CMD9, 0) == 0 && SD_RxDataBlock(ptr, 16)) res = RES_OK;
            break;
        case MMC_GET_CID:
            /* SEND_CID */
            if (SD_SendCmd(CMD10, 0) == 0 && SD_RxDataBlock(ptr, 16)) res = RES_OK;
            break;
        case MMC_GET_OCR:
            /* READ_OCR */
            if (SD_SendCmd(CMD58, 0) == 0)
            {
                for (n = 0; n < 4; n++)
                {
                    *ptr++ = SPI_RxByte();
                }
                res = RES_OK;
            }
            // Correccion: finalizar el caso antes de default
            // Evitar que el resultado sea reemplazado por RES_PARERR.
            break; 
        default:
            res = RES_PARERR;
        }
        DESELECT();
        SPI_RxByte();
    }
    return (SD_SPI_Error == HAL_OK) ? res : RES_ERROR;
}

/* Diagnostico opcional: llamar despues de finalizar una operacion FatFs. */
void SD_PrintLastDebug(void)
{
    char line[100];
    int n = snprintf(line, sizeof(line), "SD RESP:%02X FINAL:%02X RESULT:%u SPI:%d\r\n",
                     SD_DebugResponse, SD_DebugFinal, SD_DebugResult, (int)SD_SPI_Error);
    if (n > 0 && n < (int)sizeof(line))
        HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)n, 100);
}

/* USER CODE END 0 */
