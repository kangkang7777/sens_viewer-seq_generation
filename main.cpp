/******************************************************************
This file is part of https://github.com/martinruenz/sens-viewer

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as
published by the Free Software Foundation, version 3.

This program is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.
*****************************************************************/
#define NOMINMAX
#define WIN32
#include <cstddef>
#include <iostream>
#include "external/sensorData.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

typedef std::chrono::time_point<std::chrono::system_clock> Timepoint;
typedef std::chrono::duration<double> Duration;
typedef std::chrono::system_clock Clock;
vector<cv::Mat> v_depth; //存储depth
vector<cv::Mat> v_color; //存储rgb
int depth_ratio = 1000; //深度的比例
int img_height = 480; //图片高度
int img_width = 640; //图片宽度

void cutFrames(ml::SensorData &sd, int idxBegin, string filename);
int main(int argc, char* argv[])
{
	bool isSaveFile = true;

    try {
        //non-cached read
        string filename;
        if (argc == 2) filename = string(argv[1]);
        else {
            cout << "use ./sens-viewer <filename>" << endl;
            return 1;
        }
		//Each sequence contains:
		//	Color frames(frame - XXXXXX.color.jpg) : RGB, 24 - bit, JPG
		//	Depth frames(frame - XXXXXX.depth.png) : depth(mm), 16 - bit, PNG(invalid depth is set to 0)
		//	Camera poses(frame - XXXXXX.pose.txt) : camera - to - world(invalid transforms - INF)
		//	Camera calibration(info.txt) : color and depth camera intrinsics and extrinsics.
        cout << "Loading data ... ";
        cout.flush();
        ml::SensorData sd(filename);
        cout << "done!" << endl;
        cout << sd << endl; //basic information

		//cutFrames(sd,2600,"cut.sens");


		size_t numFrames = sd.m_frames.size();
		if (isSaveFile)      
		{
			//string saveDir = "E:\\CAD\\data\\rgbd";
			string saveDir = ml::util::directoryFromPath(filename);

			FILE* fp;
			fp = fopen((saveDir + "./test.seq").c_str(), "wb");

			sd.saveToImages(saveDir); //output basic information
			ml::util::makeDirectory(saveDir + "color");
			ml::util::makeDirectory(saveDir + "depth");
			ml::util::makeDirectory(saveDir + "pose");
			for (size_t i = 0; i < numFrames; i++) {
				char baseFN[30];
				ml::vec3uc* colorData = sd.decompressColorAlloc(i);
				Mat rgb(sd.m_colorHeight, sd.m_colorWidth, CV_8UC3, (unsigned char*)colorData);
				cvtColor(rgb, rgb, cv::COLOR_BGR2RGB);
				sprintf(baseFN, "\\frame-%06d.color.jpg", i);
				cv::imwrite(saveDir + "color" + baseFN, rgb);

				unsigned short* depthData = sd.decompressDepthAlloc(i);
				Mat depth_raw(sd.m_depthHeight, sd.m_depthWidth, CV_16UC1, depthData);
				sprintf(baseFN, "\\frame-%06d.depth.png", i);
				cv::imwrite(saveDir + "depth" + baseFN, depth_raw);
				
				

				//depth_raw.convertTo(depth_raw, CV_16UC1, 1000 * 1.0 / depth_ratio);
				v_depth.push_back(depth_raw);
				v_color.push_back(rgb);


				if (1)
				{
					sprintf(baseFN, "\\frame-%06d.pose.txt", i);
					const ml::mat4f & m = sd.m_frames[i].getCameraToWorld();
					ml::SensorData::savePoseFile(saveDir + "pose" + baseFN, m);
					//std::ofstream file(saveDir + "pose" + baseFN);
					//for (size_t i = 0; i < sd.m_frames.size(); i++) 
					//{
					//	//if (m._m03 != -std::numeric_limits<float>::infinity())
					//	//	file << m._m03 << " " << m._m13 << " " << m._m23 << "\n";
					//	file <<
					//		m(0, 0) << " " << m(0, 1) << " " << m(0, 2) << " " << m(0, 3) << std::endl <<
					//		m(1, 0) << " " << m(1, 1) << " " << m(1, 2) << " " << m(1, 3) << std::endl <<
					//		m(2, 0) << " " << m(2, 1) << " " << m(2, 2) << " " << m(2, 3) << std::endl <<
					//		m(3, 0) << " " << m(3, 1) << " " << m(3, 2) << " " << m(3, 3) << std::endl
					//		;
					//}
					//file.close();
				}
			}

			fwrite(&numFrames, sizeof(int32_t), 1, fp);
			fwrite(&img_height, sizeof(int32_t), 1, fp);
			fwrite(&img_width, sizeof(int32_t), 1, fp);

			for (int i = 0; i < numFrames; i++) {

				fwrite((char*)v_depth[i].data, img_height * img_width * 2, 1, fp);
				fwrite((uchar*)v_color[i].data, img_height * img_width * 3, 1, fp);
			}
			fclose(fp);
		}

		if(0)// for test
		for (size_t i = 0; i < numFrames; i++) {

            Timepoint t = Clock::now();

            //de-compress color and depth values
            ml::vec3uc* colorData = sd.decompressColorAlloc(i);
            unsigned short* depthData = sd.decompressDepthAlloc(i);

			//sd.m_colorWidth;
			//sd.m_colorHeight;
			//sd.m_depthWidth;
			//sd.m_depthHeight;
			//float depth_in_meters = sd.m_depthShift * depthData[0];//conversion from float[m] to ushort:

            Mat depth_raw(sd.m_depthHeight, sd.m_depthWidth, CV_16UC1, depthData);
            Mat rgb(sd.m_colorHeight, sd.m_colorWidth, CV_8UC3, (unsigned char*)colorData);
            cvtColor(rgb, rgb, cv::COLOR_BGR2RGB);
			double min, max;
			cv::minMaxLoc(depth_raw, &min, &max);
            Mat depth;
       		depth_raw.convertTo(depth, CV_16UC1, 20);//      numeric_limits<ushort>::max() / max //20, ������ȿ������

            cv::imshow("Depth", depth);
            //cv::waitKey(1);
            cv::imshow("RGB", rgb);
            cv::waitKey(0);

            free(colorData);
            free(depthData);

            Duration d = Clock::now() - t;
            cout << "Playback speed: " <<  1.0f / d.count() << "Hz\r";
            cout.flush();
        }

        cout << endl;
    }catch (const exception& e){
        cout << "Exception " << e.what() << endl;
        return 1;
    }catch (...){
        cout << "Exception (unknown)";
        return 1;
    }

    return 0;
}


void cutFrames(ml::SensorData &sd, int idxBegin, string filename)
{
	ml::SensorData cutSd;
	cutSd.initDefault(sd.m_colorWidth,sd.m_colorHeight, sd.m_depthWidth, sd.m_depthHeight,
					  sd.m_calibrationColor,sd.m_calibrationDepth, sd.m_colorCompressionType, sd.m_depthCompressionType);

	for (size_t i = idxBegin; i < sd.m_frames.size(); i++) {
		cutSd.m_frames.push_back(sd.m_frames[i]);
		if (sd.m_IMUFrames.size() > 0)
			cutSd.m_IMUFrames.push_back(sd.m_IMUFrames[i]);
	}
	//auto it = sd.m_frames.begin();
	//auto itEnd = it + idxBegin;
	//sd.m_frames.erase(it, itEnd);
	//if (sd.m_IMUFrames.size() > 0)
	//{
	//	auto it = sd.m_IMUFrames.begin();
	//	auto itEnd = it + idxBegin;
	//	sd.m_IMUFrames.erase(it, itEnd);
	//}

	cutSd.saveToFile(filename);
}
