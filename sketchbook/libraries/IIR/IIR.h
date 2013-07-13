
/*
 -- Author: Osman Ali (o.ali@tum.de) 
 
 ============================================
This code is placed under the MIT license
Copyright (c) 2013 Osman Ali

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/


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
