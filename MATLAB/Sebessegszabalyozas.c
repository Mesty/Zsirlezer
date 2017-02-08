// Beavatkozo jel meredekseg korlatja [100/s]
#define MEREDEKSEG_GYORSITASNAL 5
#define MEREDEKSEG_LASSITASNAL 5

static float beavatkozo_jel = 0;
static float elozo_beavatkozo_jel = 0;
static float pozitiv_visszacsatolas = 0;
float FOXBORO_bemeno_jel = 0;
 
// Szabalyozas
pozitiv_visszacsatolas = 0.99288*pozitiv_visszacsatolas+0.0071168*elozo_beavatkozo_jel;
FOXBORO_bemeno_jel = 0.72413*(mmpersec-velocity)+pozitiv_visszacsatolas;
if(FOXBORO_bemeno_jel > 2891.7847)
	beavatkozo_jel = 2891.7847;
else if(FOXBORO_bemeno_jel < -2891.7847)
	beavatkozo_jel = -2891.7847;
else
	beavatkozo_jel = FOXBORO_bemeno_jel;
if(beavatkozo_jel - elozo_beavatkozo_jel > MEREDEKSEG_GYORSITASNAL)
	beavatkozo_jel = elozo_beavatkozo_jel + MEREDEKSEG_GYORSITASNAL;
else if(elozo_beavatkozo_jel - beavatkozo_jel > MEREDEKSEG_LASSITASNAL)
	beavatkozo_jel = elozo_beavatkozo_jel - MEREDEKSEG_LASSITASNAL;
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
motorpulsePWM = (uint32_t) (beavatkozo_jel+6932);
elozo_beavatkozo_jel = beavatkozo_jel;