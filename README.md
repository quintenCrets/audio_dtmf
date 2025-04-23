# DTMF OEFENING

## youtube

## DTMF generator
De dtmf generator van het project werkt met een timer interupt die aan 8000Hz interupts genereert.
Iedere interupt update vervolgens twee variabelen waarbij de frequentie van beide sinussen wordt gëupdate.
Daarna berekent de interupt routine wat de waarde van beide sinussen opgetelt is.
De bekome waarde wordt dan nog door een moving average filter gehaalt die de slechts de actuele en vorige waarde gebruikt, en zet de uitkomst op de audio poort.

```
//www.cteq.eu
// Timer_ISR for sine generation (no LUT, our processor seems to be fast enough ;-) )
static void Timer_ISR(void *callback_ref)
{
	callback_ref_t *callback = ( callback_ref_t * )callback_ref;

	XScuTimer *timer_instance_ptr = ( XScuTimer * )callback->timer_instance_ptr;
	float32_t frequency1 = *( ( float32_t * )callback->frequency1 );
	float32_t frequency2 = *( ( float32_t * )callback->frequency2 );

	XScuTimer_ClearInterruptStatus( timer_instance_ptr );
	float32_t theta_increment = 2 * PI * frequency1 / SAMPLE_RATE;
	float32_t beta_increment = 2 * PI * frequency2 / SAMPLE_RATE;
	static float32_t theta = 0.0f;
	static float32_t beta = 0.0f;
	static float32_t running_sum = 0.0f;
	static float32_t buffer[AVERAGING_BUFFER_SIZE] = { 0 };
	static uint8_t buffer_counter = 0;
	static unsigned long isr_counter = 0;


	theta += theta_increment ;
	if ( theta > 2* PI)
		theta -= 2* PI;

	beta += beta_increment ;
	if ( beta > 2* PI)
		beta -= 2* PI;

	float32_t sine_value = arm_sin_f32(theta); // CMSIS function
	sine_value += arm_sin_f32(beta);

	running_sum -= buffer[buffer_counter];
	buffer[buffer_counter] = sine_value;
	buffer_counter = ( buffer_counter + 1 ) % AVERAGING_BUFFER_SIZE;
	running_sum += sine_value;

	uint32_t scaled_value = (uint32_t)(((running_sum / AVERAGING_BUFFER_SIZE ) + 1.0f ) * UINT_SCALED_MAX_VALUE);
	Xil_Out32(I2S_DATA_TX_R_REG, scaled_value);
	++isr_counter;

	if (isr_counter == SAMPLE_RATE )
	{
		isr_flag = ( isr_flag % 12 ) + 1;
		isr_counter = 0;
	}
}
```
### notities:
- De moving average filter is in principe overbodig, deze was oorspronkelijk geïmplementeerd voor de lage resolutie ( 8000Hz ) te verbeteren. 
Dit werkte echter niet zoals verwacht, maar is in de code gebleven aangezien deze geen invloed heeft op het uitgangs signaal en er zelfs zorgt dat er "minder ruis" aanwezig is.
- code kan nog versnelt worden met een lut maar dit is niet noodzakelijk. het uitgangsignaal is goed qua vorm en bevat nagenoeg geen ruis.
De lut zou zelfs voor vertraging zorgen bij opstart aangezien de 12 verschillende dtmf tonen elk een eigen array nodig hebben van een paar 1000 samples voor naukeurig genoeg te zijn.

## DTMF analyse

