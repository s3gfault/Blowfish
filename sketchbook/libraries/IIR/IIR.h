


#include <Arduino.h>
#include <stdlib.h>
class IIR{

public:

	//IIR();
	IIR(float *fbcoeffs,float *fwcoeffs, uint8_t fborder, uint8_t fworder);

	void setForwardCoeffs(float *nfwcoeffs);
	void setForwardCoeffs(uint8_t idx, float ncoeff);	

	void setFeedbackCoeffs(float *nfbcoeffs);
	void setFeedbackCoeffs(uint8_t idx, float ncoeff);	

	void setGain(float);

	float getGain();
	float step(float x0);
	uint8_t getForwardOrder();	
	uint8_t getFeedbackOrder();	
	float *x ;
	float *y ;

	float getX(int8_t idx); // 0 -> yn; 1 -> y[n-1] and so on
	float getY(int8_t idx); // same
	
private:

	uint8_t fborder;
	uint8_t fworder;
	float *fbcoeffs ;
	float *fwcoeffs ;
	int8_t ai ;
	int8_t bj ;
	
	float gain;

};
