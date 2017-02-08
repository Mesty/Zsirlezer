// Belso valtozok
int32_t alapjel = mmpersec;
static int32_t elozo_alapjel = 0;
static float beavatkozo_jel = 0;
static float elozo_beavatkozo_jel = 0;
static float pozitiv_visszacsatolas = 0;
float FOXBORO_bemeno_jel = 0;
 
// Szabalyozas
// Szabalyozasi algoritmus
pozitiv_visszacsatolas = 0.99288*pozitiv_visszacsatolas+0.0071168*beavatkozo_jel;
FOXBORO_bemeno_jel = 0.72413*(mmpersec-velocity)+pozitiv_visszacsatolas;
// Beavatkozo szerv telites kezelese
if(FOXBORO_bemeno_jel > 1000)
	beavatkozo_jel = 1000;
else if(FOXBORO_bemeno_jel < -1000)
	beavatkozo_jel = -1000;
else
	beavatkozo_jel = FOXBORO_bemeno_jel;
// Inverz statikus nemlinearitas
if(beavatkozo_jel > 0)
{
	if((0 < beavatkozo_jel) && (beavatkozo_jel <= 112.0907))
		beavatkozo_jel = 200+0.89213*beavatkozo_jel;
	else if((112.0907 < beavatkozo_jel) && (beavatkozo_jel <= 350.2835))
		beavatkozo_jel = 300+0.41983*(beavatkozo_jel-112.0907);
	else if((350.2835 < beavatkozo_jel) && (beavatkozo_jel <= 560.4536))
		beavatkozo_jel = 400+0.47581*(beavatkozo_jel-350.2835);
	else if((560.4536 < beavatkozo_jel) && (beavatkozo_jel <= 756.6123))
		beavatkozo_jel = 500+0.50979*(beavatkozo_jel-560.4536);
	else if((756.6123 < beavatkozo_jel) && (beavatkozo_jel <= 910.737))
		beavatkozo_jel = 600+0.64883*(beavatkozo_jel-756.6123);
}
// Beavatkozo jel kiadasa
motorpulsePWM = (uint32_t) (beavatkozo_jel+6932);
// Integratorok feltoltese
elozo_alapjel = alapjel;
elozo_beavatkozo_jel = beavatkozo_jel;