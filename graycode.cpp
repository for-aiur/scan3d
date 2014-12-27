#include "graycode.h"
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <boost/math/special_functions/round.hpp>

bool CalculateGP(cv::Mat& absPhase, std::vector<cv::Mat> &images, int startGray, int endGray, int startPhase)
{
    cv::Mat gray_code(absPhase.rows, absPhase.cols, CV_8UC1);
    cv::Mat phase(absPhase.rows, absPhase.cols, CV_8UC1);
    cv::Mat mask(absPhase.rows, absPhase.cols, CV_8UC1);

    if(!CalculateGrayCodeImg(gray_code, images, startGray, endGray))
    {
        std::cout << "Error creating gray code image";
        return false;
    }
    cv::imwrite("graycode.jpg", gray_code);

    CalculateAbsolutePhase(phase, images, startPhase);

    cv::imwrite("phase.jpg", phase);

    MaskEvaluation(mask, images[startPhase], images[startPhase+1], images[startPhase+2], images[startPhase+3], 5, 255, 10); //defults
    BinaryAndOperation(phase, mask);
    BinaryAndOperation(gray_code, mask);

    cv::imwrite("phase2.jpg", phase);

    EvaluateAbsPhase(absPhase, phase, gray_code);

    return false;
}

bool CalculateGrayCodeImg( cv::Mat& code_img, std::vector<cv::Mat>& images, long StartIndex, long EndIndex )
{
    unsigned char *normal = NULL;
    unsigned char *invers = NULL;
    unsigned char *result = NULL;
    long           j, h, BitPlane;
    unsigned long  k;

    // Check if we have correct amount of images
    if( static_cast<long>(images.size()) <= EndIndex || (EndIndex-StartIndex+1)!=16 )
    {
        std::cout << "not enough images provided\n";
        return false;
    }

    for( int i=StartIndex; i<=EndIndex; i++ )
    {
        if( images[i].type() != CV_8UC1 )
        {
            std::cout << "Wrong image type\n";
            return false;
        }
    }

    unsigned int width = images[0].cols;
    unsigned int height = images[0].rows;

    // Image size and format test
    if( code_img.type() != CV_8UC1 || code_img.cols != width || code_img.rows != height )
    {
        std::cout << "wrong format of gray image\n";
        return false;
    }

    for( unsigned int i=0; i < height; i++ )
    {
        result = (unsigned char*)code_img.row(i).data;
        memset(result, 0, code_img.cols );

        for( j=0; j<=7; j++ )
        {
            normal = (unsigned char*)images[2*j+StartIndex].row(i).data;
            invers = (unsigned char*)images[2*j+1+StartIndex].row(i).data;
            BitPlane = 1 << (7-j);

            for( k=0; k < width; k++ )
            {
                if (normal[k] > invers[k]){
                    result[k] = (unsigned char)( result[k] | BitPlane );
                }
            }
        }

        for( k=0; k < width; k++ )
        {
            h = result[k];
            // inverse graycode calculation
            result[k] = (unsigned char)( LinearCode( h & 255 ) );
        }
    }
    return true;
}

bool CalculateAbsolutePhase(cv::Mat& phase_img, std::vector<cv::Mat>& images, int offset)
{
    unsigned char *P;
    unsigned char *Q;
    unsigned char *S;
    unsigned char *T;
    unsigned char *result;

    long	i,j;
    long	A,B;
    double	Scale;
    long	ImgWidth, ImgHeight;

    ImgWidth = images[0].cols;
    ImgHeight = images[0].rows;

    if( phase_img.type() != CV_8UC1 || phase_img.cols != ImgWidth || phase_img.rows != ImgHeight )
    {
        std::cout << "wrong format of phase image";
        return false;
    }

    Scale = 128.0/M_PI;

    // all rows
    for( i=0; i<ImgHeight; i++ )
    {
        P = images[offset+0].row( i ).data;
        Q = images[offset+1].row( i ).data;
        S = images[offset+2].row( i ).data;
        T = images[offset+3].row( i ).data;
        result = (unsigned char*)phase_img.row( i ).data;
        // all columns
        for( j=0; j<ImgWidth; j++ ) {
            A = S[j] - P[j];
            B = T[j] - Q[j];
            if ((A == 0) && (B == 0)) result[j] = (unsigned char)(0);
            else result[j] = (unsigned char)boost::math::lround(atan2((double)A, (double)B)*Scale);
        }
    }
    return true;
}

void MaskEvaluation(cv::Mat& Mask, const cv::Mat& Phase1, const cv::Mat& Phase2, const cv::Mat& Phase3, const cv::Mat& Phase4,
                           int DynamicThreshold, int MaximumThreshold, int SinusThreshold )
{
    unsigned char* P;
    unsigned char* Q;
    unsigned char* S;
    unsigned char* T;
    unsigned char* R;
    long W,H,I,J;
    long Min,Max;

    W = (long)(Mask.cols);
    H = (long)(Mask.rows);

    for (I = 0; I < H; I++)
    {
        P = (unsigned char*) Phase1.row(I).data;
        Q = (unsigned char*) Phase2.row(I).data;
        S = (unsigned char*) Phase3.row(I).data;
        T = (unsigned char*) Phase4.row(I).data;
        R = (unsigned char*) Mask.row(I).data;

        for (J = 0; J < W; J++)
        {
            Min = (*P);
            Max = (*P);
            if ((*Q) < Min) Min = (*Q); else if ((*Q) > Max) Max = (*Q);
            if ((*S) < Min) Min = (*S); else if ((*S) > Max) Max = (*S);
            if ((*T) < Min) Min = (*T); else if ((*T) > Max) Max = (*T);

            if( (Max-Min)>=DynamicThreshold )
            {
                if( MaximumThreshold==255 ) {
                    if( (*P) <MaximumThreshold && (*Q)<MaximumThreshold && (*S)<MaximumThreshold && (*T)<MaximumThreshold )
                        *R = (unsigned char)(255);
                    else if( abs( ( (*P) + (*S) ) - ( (*Q) + (*T) ) ) <= SinusThreshold )	//ist die Phase sinusf\F6rmig?
                        *R = (unsigned char)(255);
                    else
                        *R = (unsigned char)(0);
                }
                else {
                    if( (*P)<=MaximumThreshold && (*Q)<=MaximumThreshold && (*S)<=MaximumThreshold && (*T)<=MaximumThreshold )
                        *R = (unsigned char)(255);
                    else
                        *R = (unsigned char)(0);
                }
            }
            else *R = (unsigned char)(0);

            P++; Q++; S++; T++; R++;
        }
    }
}

unsigned int BinaryAND(cv::Mat& dst, const cv::Mat& src)
{
    unsigned char *S;
    unsigned char *D;
    long i;
    long j;
    long H, W;
    unsigned int errval;

    W = (long)( dst.cols );
    H = (long)( dst.rows );

    // rows
    for( i = 0; i < H; i++)
    {
        D = (unsigned char*) dst.row(i).data;
        S = (unsigned char*) src.row(i).data;
        // cols
        for (j = 0; j < W; j++)
        {
            unsigned char dval = (D[j]);
            unsigned char sval = (S[j]);
            D[j] = dval & sval;
            //D[j] &= S[j];
        }
    }
    return 0;
}


void EvaluateAbsPhase(cv::Mat& AbsPhase, cv::Mat& Phase, cv::Mat& GCode)
{
    unsigned char* GC;
    unsigned char* Ph;
    unsigned short* APh;
    long I,j;
    long c,p,PRI;
    long W,H;

    W = (long)(GCode.cols);
    H = (long)(GCode.rows);

    PRI = 64;

    for(j=0; j < H  ; j++)
    {
        GC = (unsigned char*)GCode.row(j).data;
        Ph = (unsigned char*)Phase.row(j).data;
        APh = (unsigned short*)AbsPhase.row(j).data;

        for (I = 0; I < W; I++)
        {
            c = (*GC);
            p = (char)(*Ph);
            if ((c < 1)||(c > 254))
            {
                *APh = 0;
            }
            else
            {
                if (p < -PRI)
                {
                    if (c & 1)
                    {
                        c++;
                    }
                }
                else
                {
                    if (p > PRI)
                    {
                        if (!(c & 1))
                        {
                            c--;
                        }
                    }
                }
                *APh = (unsigned short)(((c >> 1) << 8) + p);
            }
            GC++; Ph++; APh++;
        }// endfor
    }// end for j
}

unsigned long LinearCode(unsigned long n)
{
    unsigned long idiv;
    int ish = 1;
    unsigned long ans = n;

    for(;;)
    {
        ans ^= ( idiv = ans >> ish );
        if( idiv <= 1 || ish == 16 )
            return ans;
        ish <<= 1;
    }
}

double BLInterpolate(double x, double y, const cv::Mat &Phase)
{
    //bilinear interpolation
    double dx = x-(int)x;
    double dy = y-(int)y;

    short ptl = Phase.at<short>((int)y , (int)x);
    short ptr = Phase.at<short>((int)y, (int)x+1);
    short pbl = Phase.at<short>((int)y+1, (int)x);
    short pbr = Phase.at<short>((int)y+1, (int)x+1);

    double weight_tl = (1.0 - dx) * (dy);
    double weight_tr = (dx)       * (dy);
    double weight_bl = (1.0 - dx) * (1.0 - dy);
    double weight_br = (dx)       * (1.0 - dy);

    return (ptl*weight_tl)+(ptr*weight_tr)+(pbl*weight_bl)+(pbr*weight_br);
}

unsigned int BinaryAndOperation(cv::Mat& dst, const cv::Mat& src)
{
    unsigned char *S;
    unsigned char *D;
    long i;
    long j;
    long H, W;
    unsigned int errval;

    // check image compatibility
    //if( (errval = CompatImages( dst, src, PIX_8BIT ) ) != OK ) return errval;

    W = (long)( dst.cols );
    H = (long)( dst.rows );

    // rows
    for( i = 0; i < H; i++)
    {
        D = (unsigned char*) dst.row(i).data;
        S = (unsigned char*) src.row(i).data;
        // cols
        for (j = 0; j < W; j++)
        {
            unsigned char dval = (D[j]);
            unsigned char sval = (S[j]);
            D[j] = dval & sval;
            //D[j] &= S[j];
        }
    }
    return 0;
}
