

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




#include <IIR.h>

//class IIR{

IIR::IIR(float *fbcoeffs,float *fwcoeffs, uint8_t nfborder, uint8_t nfworder){

	setForwardCoeffs(fwcoeffs);
	setFeedbackCoeffs(fbcoeffs);

	fborder = nfborder+1;
	fworder = nfworder+1;
	gain = 1.00f;
	x = (float*) calloc(fworder,sizeof(float));
	y = (float*) calloc(fborder,sizeof(float));
}//IIR


// coeffs are in the form:[a0,a1,...,an]
void IIR::setFeedbackCoeffs(float * ncoeffs){



	if(ncoeffs[0] == 0.00f){

		fbcoeffs = ncoeffs;
		fbcoeffs[0] = 1.00f;

	}else{

		fbcoeffs = ncoeffs;
	}




}//setFeedbackCoeffs



// coeffs are in the form:[b0,b1,...,bn]
void IIR::setForwardCoeffs(float * ncoeffs){


fwcoeffs = ncoeffs;



}//setForwardCoeffs


void IIR::setFeedbackCoeffs(uint8_t idx, float ncoeff){



	if((idx==0 )&& (ncoeff == 0.00f) ){

		fbcoeffs[idx] = 1.00f;
	}else{

		fbcoeffs[idx] = ncoeff;
	}




}//setFeedbackCoeffs

void IIR::setForwardCoeffs(uint8_t idx, float ncoeff){


fwcoeffs[idx] = ncoeff;


}//setForwardCoeffs

uint8_t IIR::getFeedbackOrder(){
return fborder;

}//getFeedbackOrder

uint8_t IIR::getForwardOrder(){
return fworder;

}//getForwardOrder

void IIR::setGain(float ngain){
gain = ngain;

}//setGain


float IIR::getGain(){
return gain;
}//getGain



float IIR::step(float x0 ){
float yn = 0;

x[bj] = x0;

	// calculate feedforward
	for(int k = 0 ; k < fworder; k++){
		yn += fwcoeffs[k] * x[(int)(((bj - k)<0)?(fworder + (bj - k)):(bj - k))];


	}
bj = (bj+1) % fworder;

	for(int k = 1 ; k <fborder; k++){
		yn += fbcoeffs[k] * y[ (int) ( ( (ai - k)<0 )?(fborder + (ai - k)):(ai - k)  )];

	}
yn/=fbcoeffs[0];  // -- y/a0

yn*=gain; // variable gain

y[ai] = yn;

ai = (ai + 1)%fborder;

return yn;
}//step


float IIR::getX(int8_t idx){
if(idx >= fworder){
idx = fworder-1;
}

return x[(int)(((bj - idx - 1)<0)?(fworder + (bj - idx -1)):(bj - idx -1))];


}//getX

float IIR::getY(int8_t idx){
if(idx >= fborder){
idx = fborder-1;
}

return y[(int)(((ai - idx - 1)<0)?(fborder + (ai - idx -1)):(ai - idx -1))];


}//getY

 



