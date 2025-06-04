# Stm32f407g-disc1-Motionsensor-Application


 
#define N 10
#define SENSITIVITY 1024
#define MULTIPLIER ((1000 << N) / SENSITIVITY)

 void init_leds(){

	   //LED3 -> PD13 CH2 ABOVE
	   //LED4 -> PD12 CH1 LEFT
	   //LED5 -> PD14 CH3 RIGHT
	   //LED6 -> PD15 CH4 DOWN

	   //Power Up Port D
	   RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

	   //Power up TIM4
	   RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

	   //Cleanup
	   GPIOD->MODER &= ~ GPIO_MODER_MODER13_Msk;
	   GPIOD->MODER &= ~ GPIO_MODER_MODER12_Msk;
	   GPIOD->MODER &= ~ GPIO_MODER_MODER15_Msk;
	   GPIOD->MODER &= ~ GPIO_MODER_MODER14_Msk;

	   //Alternate functions
	   GPIOD->MODER |= GPIO_MODER_MODE13_1;
	   GPIOD->MODER |= GPIO_MODER_MODE12_1;
	   GPIOD->MODER |= GPIO_MODER_MODE15_1;
	   GPIOD->MODER |= GPIO_MODER_MODE14_1;

	   //AF MAPPING
		GPIOD -> AFR[1] |= GPIO_AFRH_AFRH5_1;
		GPIOD -> AFR[1] |= GPIO_AFRH_AFRH4_1;
		GPIOD -> AFR[1] |= GPIO_AFRH_AFRH7_1;
		GPIOD -> AFR[1] |= GPIO_AFRH_AFRH6_1;

		TIM4->PSC = 1000;
		TIM4->ARR = 999;

		TIM4->CCR1 = 0;
		TIM4->CCR2 = 0;
		TIM4->CCR3 = 0;
		TIM4->CCR4 = 0;



		//CH1
		TIM4->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
		TIM4->CCMR1 |= TIM_CCMR1_OC1M_1;
		TIM4->CCMR1 |= TIM_CCMR1_OC1M_2;
		TIM4->CCMR1 &= ~TIM_CCMR1_OC1PE_Msk;
		TIM4->CCMR1 |= TIM_CCMR1_OC1PE;
		//CH2
		TIM4->CCMR1 &= ~TIM_CCMR1_OC2M_Msk;
		TIM4->CCMR1 |= TIM_CCMR1_OC2M_1;
		TIM4->CCMR1 |= TIM_CCMR1_OC2M_2;
		TIM4->CCMR1 &= ~TIM_CCMR1_OC2PE_Msk;
		TIM4->CCMR1 |= TIM_CCMR1_OC2PE;

		//CH3
		TIM4->CCMR2 &= ~TIM_CCMR2_OC3M_Msk;
		TIM4->CCMR2 |= TIM_CCMR2_OC3M_1;
		TIM4->CCMR2 |= TIM_CCMR2_OC3M_2;
		TIM4->CCMR2 &= ~TIM_CCMR2_OC3PE_Msk;
		TIM4->CCMR2 |= TIM_CCMR2_OC3PE;
		//CH4
		TIM4->CCMR2 &= ~TIM_CCMR2_OC4M_Msk;
		TIM4->CCMR2 |= TIM_CCMR2_OC4M_1;
		TIM4->CCMR2 |= TIM_CCMR2_OC4M_2;
		TIM4->CCMR2 &= ~TIM_CCMR2_OC4PE_Msk;
		TIM4->CCMR2 |= TIM_CCMR2_OC4PE;





		//CH1
		TIM4->CCER &= ~TIM_CCER_CC1E_Msk;
		TIM4->CCER |= TIM_CCER_CC1E;

		//CH2
		TIM4->CCER &= ~TIM_CCER_CC2E_Msk;
		TIM4->CCER |= TIM_CCER_CC2E;

		//CH3
		TIM4->CCER &= ~TIM_CCER_CC3E_Msk;
		TIM4->CCER |= TIM_CCER_CC3E;

		//CH4
		TIM4->CCER &= ~TIM_CCER_CC4E_Msk;
		TIM4->CCER |= TIM_CCER_CC4E;

		// TIM4->CR1 &=~TIM_CR1_ARPE_Msk;
		TIM4->CR1 &=~TIM_CR1_ARPE;

		TIM4->EGR &=~TIM_EGR_UG;

		TIM4->CR1 |= TIM_CR1_CEN;

		TIM4->CCR1 = 0; //PD12
		TIM4->CCR2 = 0; //PD13
		TIM4->CCR3 = 0; //PD14
		TIM4->CCR4 = 0; //PD15
		DBGMCU->APB1FZ &= ~DBGMCU_APB1_FZ_DBG_TIM4_STOP;
    }


	void init_SPI(){


			RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;


			//Power up port A

			RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;


			//Power up port E

			RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

			//NSS configuration

			GPIOE -> MODER &= ~GPIO_MODER_MODER3_Msk;


			GPIOE -> MODER |= GPIO_MODER_MODER3_0;



			//SCK Configuration

			GPIOA -> MODER &= ~GPIO_MODER_MODER5_Msk;


			GPIOA -> MODER |= GPIO_MODER_MODER5_1;



			//MISO CONFIG

			GPIOA -> MODER &= ~GPIO_MODER_MODER6_Msk;


			GPIOA -> MODER |= GPIO_MODER_MODER6_1;


			//MOSI CONFIG


			GPIOA -> MODER &= ~GPIO_MODER_MODER7_Msk;


			GPIOA -> MODER |= GPIO_MODER_MODER7_1;






			//AF MAPPING TO PORTS


			GPIOE->ODR |= GPIO_ODR_OD3;  // CS HIGH AT THE BEGINNING


			//PA 5

			GPIOA -> AFR[0] |= GPIO_AFRL_AFRL5_0;

			GPIOA -> AFR[0] |= GPIO_AFRL_AFRL5_2;

			//PA6

			GPIOA -> AFR[0] |= GPIO_AFRL_AFRL6_0;

			GPIOA -> AFR[0] |= GPIO_AFRL_AFRL6_2;

			//PA7

			GPIOA -> AFR[0] |= GPIO_AFRL_AFRL7_0;

			GPIOA -> AFR[0] |= GPIO_AFRL_AFRL7_2;


			//Set Master

			SPI1 -> CR1 &= ~SPI_CR1_MSTR_Msk;

			SPI1 -> CR1 |= SPI_CR1_MSTR;


			//Polarization -> 1

			SPI1 -> CR1 &= ~SPI_CR1_CPOL_Msk;

			SPI1 -> CR1 |= SPI_CR1_CPOL;


			//Phase -> 1

			SPI1 -> CR1 &= ~SPI_CR1_CPHA_Msk;

			SPI1 -> CR1 |= SPI_CR1_CPHA;



			//Baud Rate -> FPclk/128

			SPI1 -> CR1 &= ~SPI_CR1_BR_Msk;
			SPI1 -> CR1 |= SPI_CR1_BR_1;
			SPI1 -> CR1 |= SPI_CR1_BR_2;








			//SSM AND SSI

			SPI1 -> CR1 &= ~SPI_CR1_SSM_Msk;

			SPI1 -> CR1 |= SPI_CR1_SSM;

			SPI1 -> CR1 &= ~SPI_CR1_SSI_Msk;

			SPI1 -> CR1 |= SPI_CR1_SSI;


			//DFF

			SPI1 -> CR1 &= ~SPI_CR1_DFF_Msk;

			// SPI1 -> CR1 |= SPI_CR1_DFF;



			//SPI ENABLE

			SPI1 ->CR1 &= ~SPI_CR1_SPE_Msk;

			SPI1 -> CR1 |= SPI_CR1_SPE;



			}


			uint8_t SPI1_Transmit(uint8_t data){

				while(!(SPI1 -> SR & SPI_SR_TXE)); // Until TX BUFFER EMPTY

				*(volatile uint8_t *)&SPI1 -> DR = data;

				while(!(SPI1 -> SR & SPI_SR_RXNE));// Wait for RX to complete

				return *(volatile uint8_t *)&SPI1 ->DR;

			}


			uint8_t read_whoami() {

				uint8_t id;
				GPIOE->ODR &= ~GPIO_ODR_OD3;      // CS LOW
				SPI1_Transmit(0x8F);// WHO_AM_I = 0x0F | 0x80 (read)
				id = SPI1_Transmit(0x00);
				GPIOE->ODR |= GPIO_ODR_OD3;       // CS HIGH
				return id;                        // ID (0xD3)

			}


		    void enable_axises() {

				GPIOE->ODR &= ~GPIO_ODR_OD3;      // CS LOW
				SPI1_Transmit(0x20);
				SPI1_Transmit(0x87);
				GPIOE->ODR |= GPIO_ODR_OD3;       // CS HIGH


		    }

		    void axis_settings(){

				GPIOE->ODR &= ~GPIO_ODR_OD3;      // CS LOW
				SPI1_Transmit(0x24);
				SPI1_Transmit(0x00);
				GPIOE->ODR |= GPIO_ODR_OD3;       // CS HIGH


		    }



		    void read_xyz(int16_t *x, int16_t *y, int16_t *z) {
		        uint8_t x_low, x_high, y_low, y_high, z_low, z_high;

		        // Read X axis
		        GPIOE->ODR &= ~GPIO_ODR_OD3;               // CS LOW
		        SPI1_Transmit(0x80 | 0x28);                 // Read X low
		        x_low = SPI1_Transmit(0x00);
		        GPIOE->ODR |= GPIO_ODR_OD3;                // CS HIGH

		        GPIOE->ODR &= ~GPIO_ODR_OD3;               // CS LOW
		        SPI1_Transmit(0x80 | 0x29);                 // Read X high
		        x_high = SPI1_Transmit(0x00);
		        GPIOE->ODR |= GPIO_ODR_OD3;                // CS HIGH

		        // Read Y axis
		        GPIOE->ODR &= ~GPIO_ODR_OD3;
		        SPI1_Transmit(0x80 | 0x2A);
		        y_low = SPI1_Transmit(0x00);
		        GPIOE->ODR |= GPIO_ODR_OD3;

		        GPIOE->ODR &= ~GPIO_ODR_OD3;
		        SPI1_Transmit(0x80 | 0x2B);
		        y_high = SPI1_Transmit(0x00);
		        GPIOE->ODR |= GPIO_ODR_OD3;

		        // Read Z axis
		        GPIOE->ODR &= ~GPIO_ODR_OD3;
		        SPI1_Transmit(0x80 | 0x2C);
		        z_low = SPI1_Transmit(0x00);
		        GPIOE->ODR |= GPIO_ODR_OD3;

		        GPIOE->ODR &= ~GPIO_ODR_OD3;
		        SPI1_Transmit(0x80 | 0x2D);
		        z_high = SPI1_Transmit(0x00);
		        GPIOE->ODR |= GPIO_ODR_OD3;

		        // Combine high and low bytes
		        *x = (int16_t)((x_high << 8) | x_low) >> 4;
		        *y = (int16_t)((y_high << 8) | y_low) >> 4;
		        *z = (int16_t)((z_high << 8) | z_low) >> 4;
		    }


			init_SPI();  //Spi and Gpio settings

			uint8_t whoami = read_whoami();

			if (whoami==0x3F){

				enable_axises();
				axis_settings();
			}
			else{

				printf("Problem!");
			}

			init_leds(); //Leds


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

    int16_t x_raw, y_raw, z_raw;
    read_xyz(&x_raw, &y_raw, &z_raw);



    uint32_t x_mg = ((int32_t)x_raw * MULTIPLIER) >> N;
    uint32_t y_mg = ((int32_t)y_raw * MULTIPLIER) >> N;
    uint32_t z_mg = ((int32_t)z_raw * MULTIPLIER) >> N;

    TIM4->CCR1 = abs(x_mg);
    TIM4->CCR2 = abs(y_mg);
    TIM4->CCR3 = abs(x_mg);
    TIM4->CCR4 = abs(y_mg);

    printf("X:%lu Y:%lu Z:%lu \n", x_mg, y_mg, z_mg);

    HAL_Delay(100);


  }
