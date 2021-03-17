/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
unsigned char lut[256] =
{
		0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
		0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
		0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
		0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
		0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
		0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
		0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
		0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
		0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
		0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
		0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
		0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
		0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
		0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
		0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
		0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
		0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
		0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
		0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
		0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
		0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
		0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
		0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
		0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
		0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
		0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
		0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
		0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
		0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
		0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
		0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
		0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};
#define USART_TXBUF_LEN 1512 //dl buf nadawczego
#define USART_RXBUF_LEN 128 // dl buf odbiorczego
uint8_t USART_TxBuf[USART_TXBUF_LEN];
uint8_t USART_RxBuf[USART_RXBUF_LEN];

//wskazniki
//__IO = volatile informacja dla kompilatora, zeby nie wczytywal zmiennej do rejestru
__IO int USART_TX_Empty=0;
__IO int USART_TX_Busy=0;
__IO int USART_RX_Empty=0;
__IO int USART_RX_Busy=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */

uint8_t USART_buffisntEMPTY(){ // funkcja sprawdzajaca czy buffor nie jest pusty
	if(USART_RX_Empty==USART_RX_Busy){
		return 0;
	}else{
		return 1;
	}
}

uint8_t USART_getchar(){ // pobranie znaku
	uint8_t tmp;
	if(USART_RX_Empty!=USART_RX_Busy){ //jak bufor nie jest pusty to
		tmp=USART_RxBuf[USART_RX_Busy]; //zapisz wartosc z bufora do zmiennej temp(tam gdzie pokazuje wskaznik zajetosci)
		USART_RX_Busy++; //przesun wskaznik zajetosci
		if(USART_RX_Busy >= USART_RXBUF_LEN)USART_RX_Busy=0; //jak wyjdzie poza zakres to cofnij do poczatku
		return tmp; // zwroc wartosc
	}else return 0;
}



uint8_t USART_getline(char *buf){ //zwraca dlugosc odebranego ciagu|arg-wskaznik do miejsca/tablicy do ktorej przekopiowany odbior
	static uint8_t bf[128]; // bufor na odbierane znaki
	static uint8_t idx=0; // index aktualnie odbieranego znaku
	static uint8_t znalezionoPoczatek = 0;
	static uint8_t wykrytoKodowanie = 0;
	uint8_t ret;
	while(USART_buffisntEMPTY()){
		bf[idx] = USART_getchar();
		if(bf[idx]==33) // jak znaleziono znak poczatku to czyścimy bufor  i zerujemy index.
		{					// następnie w linii 168 index jest inkrementowany dopoki nie znajdziemy  konca ramki
			znalezionoPoczatek=1;	// lub zerowany ponownie gdy przekrocznymy max rozmiar ramki lub zapetlimy bufor
			memset(bf,0,sizeof(bf));
			idx=0;

		}else if(bf[idx]==63 && znalezionoPoczatek == 1 && wykrytoKodowanie == 0){ //sprawdzamy czy znaleziono znak ? mowiacy
																				// o tym czy znaleziono kodowanie
			wykrytoKodowanie = 1;

		}else if(wykrytoKodowanie == 1){	//jesli znaleziono kodowanie to wchodzimy tam w nastepnym kroku zeby poprawnie nadpisac
			wykrytoKodowanie =0;																					//znak kodujacy

			if(bf[idx]==119){   //podmiana poszczegolnych znakow kodujacych na wlasciwe
				bf[idx]=33;
				idx++;
			}else if(bf[idx]==104){
				bf[idx]=34;
				idx++;
			}else if(bf[idx]==112){
				bf[idx]=63;
				idx++;
			}
			else{
					//gdy znak kodujacy  nie bedzie pokrywal sie z zadnym z protokolu to odrzucamy ramke
				idx=0;
				znalezionoPoczatek = 0;
			}

		}else if(bf[idx]==34 && znalezionoPoczatek == 1){ //gdy znaleziono koniec i poczatek ramki
			znalezionoPoczatek = 0;						//ustawiamy znaleziono poczatek na zero, czyscimy bufor i kopiujemy
			memset(buf,0,128);																	//do niego znalezione znaki
			memcpy(buf,bf,idx);
			ret=idx;
			idx=0;
			return ret; //zwracamy dlugosc ramki
		}else{
			idx++;
			if(idx>=128 || (znalezionoPoczatek == 1 && idx>110)){
				znalezionoPoczatek=0;
				idx=0;
			}
		}
	}return 0;
}

void USART_send(char* format,...){ //foramt jak w printf
	char tmp_rs[128]; //zmienna na stringa/ uwazac zeby ramki dluzszej nie dac
	int i;
	__IO int idx;  //
	va_list arglist; //argument list
	va_start(arglist,format); // kojarzenie formatu z lista arg
	vsprintf(tmp_rs,format,arglist); // do zmiennej tmp rs zgodnie z formatem i arglist drukuwanie stringa
	va_end(arglist); // kasowanie arglista
	// w tmp_rs mamy juz sformatowanego stringa
	idx=USART_TX_Empty; //zeby na raz dopisac do buff all
	for(i=0;i<strlen(tmp_rs);i++){
		USART_TxBuf[idx]=tmp_rs[i];//wrzucanie znakow do buf kolowego
		idx++;
		if(idx>=USART_TXBUF_LEN)idx=0;
	}
	__disable_irq();//wylaczamy przerwania, bo nizej sprawdzamy stan wskaznikow
	if((USART_TX_Empty==USART_TX_Busy)&&(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TXE)==SET)){ //bufor nadajnika, sprawdzamy czy na pewno ostatni znak nie jest wysylany
		USART_TX_Empty=idx;
		uint8_t tmp = USART_TxBuf[USART_TX_Busy];
		USART_TX_Busy++;
		if(USART_TX_Busy >= USART_TXBUF_LEN)USART_TX_Busy=0;
		HAL_UART_Transmit_IT(&huart2, &tmp, 1);
	}else{
		USART_TX_Empty=idx;
	}
	__enable_irq();
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart==&huart2){
		if(USART_TX_Empty!=USART_TX_Busy){
			uint8_t tmp=USART_TxBuf[USART_TX_Busy]; //wartosc pierwszego zajętego miejsca do tmp
			USART_TX_Busy++;
			if(USART_TX_Busy>=USART_TXBUF_LEN) USART_TX_Busy=0;

			HAL_UART_Transmit_IT(&huart2, &tmp,1); // wysyla kolejny znak
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){ //callback ktory odpowiada za wykrycie ze przyszly dane
	if(huart==&huart2){
		USART_RX_Empty++;
		if(USART_RX_Empty>=USART_RXBUF_LEN)USART_RX_Empty=0;
		HAL_UART_Receive_IT(&huart2,&USART_RxBuf[USART_RX_Empty],1); //zeby odebralo nastepny znak
	}
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint32_t ustal_sektor(uint32_t address)
{
  uint32_t sector = 0;
  if((address < 0x0800BFFF) && (address >= 0x08008000))
    {
      sector = FLASH_SECTOR_2;
    }
  else if((address < 0x0800FFFF) && (address >= 0x0800C000))
  {
    sector = FLASH_SECTOR_3;
  }

  return sector;
}
//Sector 2 0x08008000 - 0x0800BFFF
//Sector 3 0x0800C000 - 0x0800FFFF
uint32_t Flash_Write_Temp (uint32_t StartSectorAddress, uint32_t * temp)
{

	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SECTORError;
	int cnt=0;

	int numberofwords = (strlen(temp)/4) + ((strlen(temp) % 4) != 0);

	  HAL_FLASH_Unlock(); //odblokowanie flash
	  uint32_t StartSector = ustal_sektor(StartSectorAddress);

	  if(StartSectorAddress >=0x0800FFF0){ // ostatni pomiar bedzie w tym miejscu,
		  StartSectorAddress ==0x08008000; // ustawiam sector z powrotem na 2 zeby nie weszlo na 4.
	  }//jak zaczynamy pisac od poczatku jednego z sektorow to czyscimy go gdy nie jest pusty.
	  if(((StartSectorAddress ==0x08008000 ||StartSectorAddress ==0x0800C000 )) && (*( uint32_t *)StartSectorAddress != 0xffffffff))
	  {
		  //do wymazania sektora
			  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
			  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3; //od 2.7 do 3.6?
			  EraseInitStruct.Sector = StartSector; //od jakiego sektora wymazac
			  EraseInitStruct.NbSectors = 1; // ile sektorow wymazac

			  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
			  {
				  return HAL_FLASH_GetError ();
			  }
	  }

	   while (cnt<numberofwords)
	   {
	     if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartSectorAddress, temp[cnt]) == HAL_OK)
	     {
	    	 StartSectorAddress += 4; // bo wpisujemy slowo
	    	 cnt++; //zwiekszajac o 1 przeuwamy sie o 4 bajty
	     }
	     else
	     {
	    	 return HAL_FLASH_GetError (); // error przy pisaniu
	     }
	   }


	  HAL_FLASH_Lock();

	   return 0;
}

void delay(uint16_t time)  // opoznienie w mikrosekundach
{
	__HAL_TIM_SET_COUNTER(&htim10,0);// ustawienie timera na zero
	while((__HAL_TIM_GET_COUNTER(&htim10))<time); // i odczekanie az doliczy do podanej wartosci

}
void Set_As_Output(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct ={0}; //wyzerowanie
	GPIO_InitStruct.Pin = GPIO_Pin; //przypisanie przekazanego pinu
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // ustawienie na pushpull
	GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_LOW; //speed pinu na low
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct); // inicjalizacja zgodnie z parametrami wyzej


}
void Set_As_Input(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct ={0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT; //input
	GPIO_InitStruct.Pull = GPIO_PULLUP; //pullup zeby do jedynki podciagac
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct); //inicjalizacja zgodnie z parametrami powyzej


}
uint8_t start() //inicjalizacja Figure15, strona 15 w datasheet
{
	uint8_t Presence =0;
	Set_As_Output(sensor_GPIO_Port, sensor_Pin);
	HAL_GPIO_WritePin(sensor_GPIO_Port, sensor_Pin, 0);
	delay(480); // master musi utrzymac przez tyle linie na 0
	Set_As_Input(sensor_GPIO_Port,sensor_Pin); //zmiana na input w oczekiwaniu na presence
	delay(80); // czekamy a potem pobieramy odpowiedz
	if(!(HAL_GPIO_ReadPin(sensor_GPIO_Port, sensor_Pin))) {
		Presence=1; //jesli na pinie jest 0, to presence pulse zostal wykryty
	}else Presence=0;
	delay(400); //do konca cyklu
	return Presence;
}
void sensor_write(uint8_t data)
{
	Set_As_Output(sensor_GPIO_Port,sensor_Pin);
	for(int i=0;i<8;i++){
		if((data&(1<<i))!=0)//jesli bit jest =1
		{ //wyslij 1
			Set_As_Output(sensor_GPIO_Port, sensor_Pin);
			HAL_GPIO_WritePin(sensor_GPIO_Port,sensor_Pin, 0);
			delay(1);//datasheet strona 16   // output piszemy zero i czekam 1us

			Set_As_Input(sensor_GPIO_Port, sensor_Pin); // ustawiamy input (resistorpullup do 1) i czekamy
			delay(60); 																		//do konca cyklu

		}else //jak jest rowny 0 wysylamy 0
		{
			Set_As_Output(sensor_GPIO_Port, sensor_Pin);
			HAL_GPIO_WritePin(sensor_GPIO_Port,sensor_Pin, 0);
			delay(60); // piszemy zero na 60us
			Set_As_Input(sensor_GPIO_Port, sensor_Pin); // i ustawiamy na input


		}

	}
}
uint8_t sensor_read()
{
	uint8_t value=0;
	Set_As_Input(sensor_GPIO_Port, sensor_Pin);
	for(int i=0;i<8;i++){
		Set_As_Output(sensor_GPIO_Port, sensor_Pin);
		HAL_GPIO_WritePin(sensor_GPIO_Port,sensor_Pin, 0); //ustawiamy na zero i czekamy
		delay(2);

		Set_As_Input(sensor_GPIO_Port, sensor_Pin);
		if(HAL_GPIO_ReadPin(sensor_GPIO_Port,sensor_Pin))
		{
			value|=1<<i;
		}
		delay(60);
	}
	return value;
}
unsigned char calc_crc(const unsigned char * data, const unsigned int size)
{
	unsigned char crc = 0;
	for ( unsigned int i = 0; i < size; ++i )
	{
		crc = lut[data[i] ^ crc];
	}
	return crc;
}

uint8_t interwal_set=0;
uint16_t max =2000;
void interwal_czasowy()
{
	static uint16_t licznik =0;
	licznik++;
	if(licznik>=max)
	{
		licznik=0;
		interwal_set=1;
	}


}
uint8_t pomiar_flaga_set=0;
uint16_t licznik2;// zerowany pozniej w funkcji nizej
void do_pomiaru()
{
		licznik2++;
		if(licznik2>=750)
		{
			licznik2=0;
			pomiar_flaga_set=1;
		}
}

void wyslijDoPc(char dopcDane[],char dopcKomenda[])
{
	int rozmiar = strlen(dopcKomenda)+strlen(dopcDane);
	char doCrcString[rozmiar+1];
	memset(doCrcString,0,sizeof(doCrcString));
	strcat(doCrcString,dopcKomenda);
	strcat(doCrcString,dopcDane);
	USART_send("!%d%s%s%c\"\r\n",strlen(dopcDane),dopcKomenda,dopcDane,calc_crc(doCrcString,strlen(doCrcString)));
}

int flaga_do_pomiaru =0;
void wykonajpomiar()
{
	uint8_t presence;
	presence = start();
	if(presence){
	sensor_write(0xCC);//skip rom
	sensor_write(0x44);//convert T inicjacja konwersji temperatury.
	flaga_do_pomiaru = 1; // flaga aby przeszlo do 2 fazy
	pomiar_flaga_set =0; // zerowanie flagi systicka
	licznik2 =0; //convert T trwa do 750ms - odliczam je sys tickiem i przechodze
							//do fazy 2 \/
	}
}
void faza2pomiaru(){
	flaga_do_pomiaru =0;
	uint8_t tempB1=0;
	int16_t temp=0;
	float temperature=0;
	uint8_t tempB2=0;
	static int counterPomiarow =0;
	int adresSektora = 0x08008000;
	uint8_t presence;
	presence = start();
	if(presence){
	sensor_write(0xCC);
	sensor_write(0xBE);//read scratchpad (od najmniej znaczącego bitu)
	tempB1= sensor_read();
	tempB2= sensor_read();
	temp=(tempB2<<8)|tempB1;
	temperature=(float)temp/16;

	char temperatura[17];
	char counterPomiarowDoFlash[5];
	sprintf(counterPomiarowDoFlash,"%.4d",counterPomiarow);
	if (counterPomiarow == 0){ // przy wlaczeniu programu jak counter jest zero czytamy z flasha ostatni pomiar
		uint32_t temp;
		while (1)
		{
			temp = *( uint32_t *)adresSektora;
			if (temp == 0xffffffff) //jak dojedzie do konca zapisanych danych
			{
				adresSektora -= 16;	//cofamy sie do miejsca gdzie powinien być #
				if(*( uint8_t *)adresSektora == 0x23){ // jak trafiło dobrze na hash to
					adresSektora ++;//omijamy go i pobieramy wartosc pomiaru
					for(int i =0;i<4;i++)
					{
						counterPomiarowDoFlash[i] = *( uint32_t *)adresSektora;
						adresSektora ++;
					}
				}else{ // jak nie bylo to piszemy od poczatku
					counterPomiarow =0;
					}


				break;
			}
			adresSektora += 4;
		}
	}
	adresSektora = 0x08008000;
	counterPomiarowDoFlash[4]=0;
	if(counterPomiarow>2520)
	{
		counterPomiarow =0;
	}
	counterPomiarow =atoi(counterPomiarowDoFlash);
	adresSektora = adresSektora + (16 * counterPomiarow);
	counterPomiarow ++;
	sprintf(temperatura,"#%.4dTEMP%.4f",counterPomiarow,temperature);
	Flash_Write_Temp(adresSektora,temperatura);

	//////Wyslanie pobranej temperatury do pc //////
	char odane[19];
	sprintf(odane,"Temperatura:%g",temperature);
	char okomenda[]="TEMP";
	wyslijDoPc(odane,okomenda);

	}//if presence
} //faza2pomiaru

void odczytajPomiar(char *numer)
{

	char temp[8];
	char hexRepOfnumer[11];
	static uint32_t hexrep;
	sprintf(hexRepOfnumer,"0x%02x%02x%02x%02x",numer[3],numer[2],numer[1],numer[0]);
	hexRepOfnumer[10]=0;
	hexrep=strtoul(hexRepOfnumer);
	temp[7]=0;
	uint32_t adresSektora=0x08008000;
 while(1){
			if(*( uint8_t *)adresSektora == 0x23) // przed kazdym pomiarem jest #
			{
				adresSektora ++;//omijamy go i sprawdzamy czy dalej pomiar sie zgadza
				if( *( uint32_t *)adresSektora == hexrep)
				{
					adresSektora +=8;// przeskakujemy numer pomiaru i przedrostek "TEMP"
					for(int i =0;i<8;i++)
						{ // przypisanie kolejnych znaków
							temp[i] = *( uint32_t *)adresSektora;
							adresSektora ++;
						}
					break;
				}else{
					adresSektora += 15;
				}
			}else if(*( uint32_t *)adresSektora == 0xffffffff ||adresSektora >= 0x0800FFFF ){
				temp[0]=0;
				break;
			}
		}
	if(strlen(temp)<1)
	{
		sprintf(temp,"brak");
	}
	char odane[38];
	memset(odane,0,38);
	sprintf(odane,"ODCZ:Odczytana temperatura: %s",temp);
	char okomenda[]="ODCZ";
	wyslijDoPc(odane,okomenda);

}

uint8_t AINT=0; //stan interwalu, domyslnie 0
void sprawdz_komende(char* komenda,char* parametry,uint8_t dlugoscParametrow){
	if(!strcmp(komenda,"AINT")){

		if(dlugoscParametrow==0) //ta komenda musi miec parametr
		{
			char odane[]="ERROR:Brak parametru";
			char okomenda[]="RIEE";
			wyslijDoPc(odane,okomenda);
		}
		for(int i=0;i < dlugoscParametrow;i++)
		{

			if(parametry[i] < 48 || parametry[i] > 57 ) // jesli parametry to nie cyfra wypisz blad
			{

				char odane[]="ERROR:Parametr to nie liczba";
				char okomenda[]="RIEE";
				wyslijDoPc(odane,okomenda);
				break;
			}
			else if(i==dlugoscParametrow-1){
				max=atoi(parametry);
				if (max<760) //poniewaz convert t moze trwac 750ms.
				{
					char odane[]="ERROR:Interwal musi byc wiekszy nic 760ms";
					char okomenda[]="RIEE";
					wyslijDoPc(odane,okomenda);
					break;
				}
				AINT=1;
				char odane[50];
				memset(odane,0,50);
				sprintf(odane,"[INFO]: Interwal ustawiony na %sms",parametry);
				char okomenda[]="CINF";
				wyslijDoPc(odane,okomenda);
				interwal_set=0;break;
			}
		}

	}
	else if(!strcmp(komenda,"DINT")){
		if(dlugoscParametrow>0) //ta funkcja nie moze miec parametrow
		{
			char odane[]="ERROR:Ta funkcja nie przyjmuje parametrow";
			char okomenda[]="RIEE";
			wyslijDoPc(odane,okomenda);

		}else{
			AINT=0;
			char odane[]="[INFO]:Wykonywanie pomiarow wstrzymane";
			char okomenda[]="CINF";
			wyslijDoPc(odane,okomenda);

		}
	}
	else if(!strcmp(komenda,"STMS")){
		if(dlugoscParametrow>0)
				{
					char odane[]="ERROR:Ta funkcja nie przyjmuje parametrow";
					char okomenda[]="RIEE";
					wyslijDoPc(odane,okomenda);

				}else{

					wykonajpomiar();
				}


	}
	else if(!strcmp(komenda,"ODCZ")){
		for(int i=0;i < dlugoscParametrow;i++)
				{

					if(parametry[i] < 48 || parametry[i] > 57 ) // jesli parametry to nie cyfra wypisz blad
					{

						char odane[]="ERROR:Parametr to nie liczba";
						char okomenda[]="RIEE";
						wyslijDoPc(odane,okomenda);
						break;
					}else if(i==dlugoscParametrow-1){
						odczytajPomiar(parametry);
				}
				}

	}
	else if(!strcmp(komenda,"ODCL")){
		if(dlugoscParametrow>0)
		{
			char odane[]="ERROR:Ta funkcja nie przyjmuje parametrow";
			char okomenda[]="RIEE";
			wyslijDoPc(odane,okomenda);

		}else{
		char odane[38];
		memset(odane,0,38);
		char okomenda[5];
		okomenda[4]=0;
		char pobranaTemp[8];
		uint32_t adresSektora=0x08008000;
		while (1)
		{
			if (*( uint32_t *)adresSektora == 0xffffffff) //jak dojedzie do konca zapisanych danych
			{
				adresSektora -= 8;	//cofamy sie do miejsca gdzie zaczyna sie temperatura
				if(*( uint8_t *)adresSektora == 0x50){ // jak trafiło dobrze na litere P
					adresSektora ++;//omijamy i pobieramy wartosc pomiaru
					for(int i =0;i<8;i++)
					{
						pobranaTemp[i] = *( uint32_t *)adresSektora;
						adresSektora ++;
					}
					pobranaTemp[7]=0;
					sprintf(odane,"ODCL:Odczytana temperatura: %s",pobranaTemp);
					strcpy(okomenda,"ODCL");
					wyslijDoPc(odane,okomenda);
				}else{
					strcpy(odane,"ERROR:Nie ma takiego pomiaru");
					strcpy(okomenda,"RIEE");
					wyslijDoPc(odane,okomenda);
					break;
					}


				break;
			}//if
			adresSektora += 4;
		}//while
		}//else



	}
	else{

		wyslijDoPc("ERROR:Nie ma takiej komendy", "RIEE");
	}
}

void sprawdz_ramke(char *zawartosc_ramki,uint8_t dlugosc){
	while(1){ // zeby w dowolnym momencie przerwac funkcje komenda brake
		//////////Sprawdzenie długosci ramki/////////////////
		if(dlugosc<6){ // 6  poniewaz bez znaku poczatku i konca
			char odane[]="ERROR:Ramka jest za krotka";
			char okomenda[]="RIEE";
			wyslijDoPc(odane,okomenda);break;

		} //sprawdzenie czy nie jest za dluga mamy w funkcji getline

		////////Sprawdzanie kolumny długości danych///////////
		uint8_t counter =0; // pomocniczy licznik
		static char dlugosc_danych[4];
		for(int i=0;i<4;i++){ //czy to znaki ascii 0-9
			if((zawartosc_ramki[i]>=48) && (zawartosc_ramki[i]<=57))
			{
				dlugosc_danych[counter]=zawartosc_ramki[i];
				counter++;
			}else{
				dlugosc_danych[counter]=0;
				break;
			}
		}
		int dlugoscDanychInt = atoi(dlugosc_danych); // poniewaz pozniej bedzie uzywana kilka razy
		if(dlugoscDanychInt>100){ //czy wartosc miesci sie od zera do stu
			char odane[]="ERROR:Zbyt duza dlugosc danych";
			char okomenda[]="RIEE";
			wyslijDoPc(odane,okomenda);break;

		}else if (counter==0){ // to pole nie moze byc puste
			char odane[]="ERROR:Dlugosc danych nie moze byc pusta";
			char okomenda[]="RIEE";
			wyslijDoPc(odane,okomenda);break;
		}
		////////////Sprawdzanie komendy//////////////////

		char komenda[5];
		uint8_t flagaZlychZnakowKomendy=0;
		for(int i=0;i<4;i++)
		{
			komenda[i]=zawartosc_ramki[counter++]; // pobranie komendy z zawartosci ramki
			//sprawdzenie czy sklada sie z okreslonych w ramce znakow
			if((komenda[i]<65 || komenda[i]>122) ||(komenda[i]>90 && komenda[i]<97))
			{
				flagaZlychZnakowKomendy=1;
			}

		}
		if(flagaZlychZnakowKomendy==1){
			char odane[]="ERROR:Niepoprawne znaki w polu komenda";
			char okomenda[]="RIEE";
			wyslijDoPc(odane,okomenda);break;
		}
		komenda[4]=0;

		char parametry[100];//parametry w ramce to kolumna dane
		uint8_t dlugosc_parametrow = dlugosc - strlen(dlugosc_danych) - 5; //(5=cmd+crc)
		if(dlugosc_parametrow != dlugoscDanychInt){//inna odebrana dlugosc niz podana
			char odane[]="ERROR:Niepoprawna dlugosc danych";
			char okomenda[]="RIEE";
			wyslijDoPc(odane,okomenda);break;
		}
		for(int i=0;i<dlugoscDanychInt;i++)
		{
			parametry[i]=zawartosc_ramki[counter++]; // pobranie parametrow z zawartosci ramki
		}

		unsigned char crcFromFrame;
		if(dlugosc-(dlugoscDanychInt+5)>1){ //jesli crc jest dluzsze niz 1B to jest niepoprawne
			char odane[]="ERROR:Niepoprawne CRC";
			char okomenda[]="RIEE";
			wyslijDoPc(odane,okomenda);break;

		}
		crcFromFrame = zawartosc_ramki[dlugosc-1];// tu jest crc z ramki

		parametry[dlugoscDanychInt]=0;

		char doCrc[120];
		memset(doCrc,0,sizeof(doCrc));
		strcat(doCrc,komenda);
		strcat(doCrc,parametry);

		unsigned char x = calc_crc(doCrc,strlen(doCrc)); // tu liczymy crc zgodnie z schematem przedstawionym
																				//w protokole komunikacyjnym,
								//jednak ponizej zamiast tego wpisuje 'x' zeby nie liczyc crc recznie podczas korzystania
		if(crcFromFrame !='x') //z terminala
		{
			char odane[]="ERROR:Niepoprawne CRC";
			char okomenda[]="RIEE";
			wyslijDoPc(odane,okomenda);break;

		}
		//gdy pola zostaly wpisane poprawnie przechodzimy do sprawdzania poprawnosci komend.
		sprawdz_komende(komenda,parametry,dlugosc_parametrow);
		break;

	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_TIM10_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim10);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	HAL_UART_Receive_IT(&huart2,&USART_RxBuf[0],1); //inicjacja odbierania znakow

	int len=0;
	char bx[128];

	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		if((len=USART_getline(bx))>0){
			sprawdz_ramke(bx,len);

		}

		////////////////uruchomienie interwału///////////////////

		if(interwal_set && AINT==1)
		{
			wykonajpomiar();
			interwal_set=0;
		}
		/////////////////////////////////////////////////////////
		if(flaga_do_pomiaru == 1 && pomiar_flaga_set == 1)
		{
			faza2pomiaru();
			interwal_set=0; // w przypadku interwalu
		}



	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 50;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief TIM10 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM10_Init(void)
{

	/* USER CODE BEGIN TIM10_Init 0 */

	/* USER CODE END TIM10_Init 0 */

	/* USER CODE BEGIN TIM10_Init 1 */

	/* USER CODE END TIM10_Init 1 */
	htim10.Instance = TIM10;
	htim10.Init.Prescaler = 50-1;
	htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim10.Init.Period = 65535-1;
	htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM10_Init 2 */

	/* USER CODE END TIM10_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, sensor_Pin|LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : sensor_Pin LD2_Pin */
	GPIO_InitStruct.Pin = sensor_Pin|LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
