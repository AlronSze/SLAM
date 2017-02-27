#include "../inc/orb_feature.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

static int32_t kPattern[256 * 4] = {
	8,-3, 9,5/*mean (0), correlation (0)*/,
	4,2, 7,-12/*mean (1.12461e-05), correlation (0.0437584)*/,
	-11,9, -8,2/*mean (3.37382e-05), correlation (0.0617409)*/,
	7,-12, 12,-13/*mean (5.62303e-05), correlation (0.0636977)*/,
	2,-13, 2,12/*mean (0.000134953), correlation (0.085099)*/,
	1,-7, 1,6/*mean (0.000528565), correlation (0.0857175)*/,
	-2,-10, -2,-4/*mean (0.0188821), correlation (0.0985774)*/,
	-13,-13, -11,-8/*mean (0.0363135), correlation (0.0899616)*/,
	-13,-3, -12,-9/*mean (0.121806), correlation (0.099849)*/,
	10,4, 11,9/*mean (0.122065), correlation (0.093285)*/,
	-13,-8, -8,-9/*mean (0.162787), correlation (0.0942748)*/,
	-11,7, -9,12/*mean (0.21561), correlation (0.0974438)*/,
	7,7, 12,6/*mean (0.160583), correlation (0.130064)*/,
	-4,-5, -3,0/*mean (0.228171), correlation (0.132998)*/,
	-13,2, -12,-3/*mean (0.00997526), correlation (0.145926)*/,
	-9,0, -7,5/*mean (0.198234), correlation (0.143636)*/,
	12,-6, 12,-1/*mean (0.0676226), correlation (0.16689)*/,
	-3,6, -2,12/*mean (0.166847), correlation (0.171682)*/,
	-6,-13, -4,-8/*mean (0.101215), correlation (0.179716)*/,
	11,-13, 12,-8/*mean (0.200641), correlation (0.192279)*/,
	4,7, 5,1/*mean (0.205106), correlation (0.186848)*/,
	5,-3, 10,-3/*mean (0.234908), correlation (0.192319)*/,
	3,-7, 6,12/*mean (0.0709964), correlation (0.210872)*/,
	-8,-7, -6,-2/*mean (0.0939834), correlation (0.212589)*/,
	-2,11, -1,-10/*mean (0.127778), correlation (0.20866)*/,
	-13,12, -8,10/*mean (0.14783), correlation (0.206356)*/,
	-7,3, -5,-3/*mean (0.182141), correlation (0.198942)*/,
	-4,2, -3,7/*mean (0.188237), correlation (0.21384)*/,
	-10,-12, -6,11/*mean (0.14865), correlation (0.23571)*/,
	5,-12, 6,-7/*mean (0.222312), correlation (0.23324)*/,
	5,-6, 7,-1/*mean (0.229082), correlation (0.23389)*/,
	1,0, 4,-5/*mean (0.241577), correlation (0.215286)*/,
	9,11, 11,-13/*mean (0.00338507), correlation (0.251373)*/,
	4,7, 4,12/*mean (0.131005), correlation (0.257622)*/,
	2,-1, 4,4/*mean (0.152755), correlation (0.255205)*/,
	-4,-12, -2,7/*mean (0.182771), correlation (0.244867)*/,
	-8,-5, -7,-10/*mean (0.186898), correlation (0.23901)*/,
	4,11, 9,12/*mean (0.226226), correlation (0.258255)*/,
	0,-8, 1,-13/*mean (0.0897886), correlation (0.274827)*/,
	-13,-2, -8,2/*mean (0.148774), correlation (0.28065)*/,
	-3,-2, -2,3/*mean (0.153048), correlation (0.283063)*/,
	-6,9, -4,-9/*mean (0.169523), correlation (0.278248)*/,
	8,12, 10,7/*mean (0.225337), correlation (0.282851)*/,
	0,9, 1,3/*mean (0.226687), correlation (0.278734)*/,
	7,-5, 11,-10/*mean (0.00693882), correlation (0.305161)*/,
	-13,-6, -11,0/*mean (0.0227283), correlation (0.300181)*/,
	10,7, 12,1/*mean (0.125517), correlation (0.31089)*/,
	-6,-3, -6,12/*mean (0.131748), correlation (0.312779)*/,
	10,-9, 12,-4/*mean (0.144827), correlation (0.292797)*/,
	-13,8, -8,-12/*mean (0.149202), correlation (0.308918)*/,
	-13,0, -8,-4/*mean (0.160909), correlation (0.310013)*/,
	3,3, 7,8/*mean (0.177755), correlation (0.309394)*/,
	5,7, 10,-7/*mean (0.212337), correlation (0.310315)*/,
	-1,7, 1,-12/*mean (0.214429), correlation (0.311933)*/,
	3,-10, 5,6/*mean (0.235807), correlation (0.313104)*/,
	2,-4, 3,-10/*mean (0.00494827), correlation (0.344948)*/,
	-13,0, -13,5/*mean (0.0549145), correlation (0.344675)*/,
	-13,-7, -12,12/*mean (0.103385), correlation (0.342715)*/,
	-13,3, -11,8/*mean (0.134222), correlation (0.322922)*/,
	-7,12, -4,7/*mean (0.153284), correlation (0.337061)*/,
	6,-10, 12,8/*mean (0.154881), correlation (0.329257)*/,
	-9,-1, -7,-6/*mean (0.200967), correlation (0.33312)*/,
	-2,-5, 0,12/*mean (0.201518), correlation (0.340635)*/,
	-12,5, -7,5/*mean (0.207805), correlation (0.335631)*/,
	3,-10, 8,-13/*mean (0.224438), correlation (0.34504)*/,
	-7,-7, -4,5/*mean (0.239361), correlation (0.338053)*/,
	-3,-2, -1,-7/*mean (0.240744), correlation (0.344322)*/,
	2,9, 5,-11/*mean (0.242949), correlation (0.34145)*/,
	-11,-13, -5,-13/*mean (0.244028), correlation (0.336861)*/,
	-1,6, 0,-1/*mean (0.247571), correlation (0.343684)*/,
	5,-3, 5,2/*mean (0.000697256), correlation (0.357265)*/,
	-4,-13, -4,12/*mean (0.00213675), correlation (0.373827)*/,
	-9,-6, -9,6/*mean (0.0126856), correlation (0.373938)*/,
	-12,-10, -8,-4/*mean (0.0152497), correlation (0.364237)*/,
	10,2, 12,-3/*mean (0.0299933), correlation (0.345292)*/,
	7,12, 12,12/*mean (0.0307242), correlation (0.366299)*/,
	-7,-13, -6,5/*mean (0.0534975), correlation (0.368357)*/,
	-4,9, -3,4/*mean (0.099865), correlation (0.372276)*/,
	7,-1, 12,2/*mean (0.117083), correlation (0.364529)*/,
	-7,6, -5,1/*mean (0.126125), correlation (0.369606)*/,
	-13,11, -12,5/*mean (0.130364), correlation (0.358502)*/,
	-3,7, -2,-6/*mean (0.131691), correlation (0.375531)*/,
	7,-8, 12,-7/*mean (0.160166), correlation (0.379508)*/,
	-13,-7, -11,-12/*mean (0.167848), correlation (0.353343)*/,
	1,-3, 12,12/*mean (0.183378), correlation (0.371916)*/,
	2,-6, 3,0/*mean (0.228711), correlation (0.371761)*/,
	-4,3, -2,-13/*mean (0.247211), correlation (0.364063)*/,
	-1,-13, 1,9/*mean (0.249325), correlation (0.378139)*/,
	7,1, 8,-6/*mean (0.000652272), correlation (0.411682)*/,
	1,-1, 3,12/*mean (0.00248538), correlation (0.392988)*/,
	9,1, 12,6/*mean (0.0206815), correlation (0.386106)*/,
	-1,-9, -1,3/*mean (0.0364485), correlation (0.410752)*/,
	-13,-13, -10,5/*mean (0.0376068), correlation (0.398374)*/,
	7,7, 10,12/*mean (0.0424202), correlation (0.405663)*/,
	12,-5, 12,9/*mean (0.0942645), correlation (0.410422)*/,
	6,3, 7,11/*mean (0.1074), correlation (0.413224)*/,
	5,-13, 6,10/*mean (0.109256), correlation (0.408646)*/,
	2,-12, 2,3/*mean (0.131691), correlation (0.416076)*/,
	3,8, 4,-6/*mean (0.165081), correlation (0.417569)*/,
	2,6, 12,-13/*mean (0.171874), correlation (0.408471)*/,
	9,-12, 10,3/*mean (0.175146), correlation (0.41296)*/,
	-8,4, -7,9/*mean (0.183682), correlation (0.402956)*/,
	-11,12, -4,-6/*mean (0.184672), correlation (0.416125)*/,
	1,12, 2,-8/*mean (0.191487), correlation (0.386696)*/,
	6,-9, 7,-4/*mean (0.192668), correlation (0.394771)*/,
	2,3, 3,-2/*mean (0.200157), correlation (0.408303)*/,
	6,3, 11,0/*mean (0.204588), correlation (0.411762)*/,
	3,-3, 8,-8/*mean (0.205904), correlation (0.416294)*/,
	7,8, 9,3/*mean (0.213237), correlation (0.409306)*/,
	-11,-5, -6,-4/*mean (0.243444), correlation (0.395069)*/,
	-10,11, -5,10/*mean (0.247672), correlation (0.413392)*/,
	-5,-8, -3,12/*mean (0.24774), correlation (0.411416)*/,
	-10,5, -9,0/*mean (0.00213675), correlation (0.454003)*/,
	8,-1, 12,-6/*mean (0.0293635), correlation (0.455368)*/,
	4,-6, 6,-11/*mean (0.0404971), correlation (0.457393)*/,
	-10,12, -8,7/*mean (0.0481107), correlation (0.448364)*/,
	4,-2, 6,7/*mean (0.050641), correlation (0.455019)*/,
	-2,0, -2,12/*mean (0.0525978), correlation (0.44338)*/,
	-5,-8, -5,2/*mean (0.0629667), correlation (0.457096)*/,
	7,-6, 10,12/*mean (0.0653846), correlation (0.445623)*/,
	-9,-13, -8,-8/*mean (0.0858749), correlation (0.449789)*/,
	-5,-13, -5,-2/*mean (0.122402), correlation (0.450201)*/,
	8,-8, 9,-13/*mean (0.125416), correlation (0.453224)*/,
	-9,-11, -9,0/*mean (0.130128), correlation (0.458724)*/,
	1,-8, 1,-2/*mean (0.132467), correlation (0.440133)*/,
	7,-4, 9,1/*mean (0.132692), correlation (0.454)*/,
	-2,1, -1,-4/*mean (0.135695), correlation (0.455739)*/,
	11,-6, 12,-11/*mean (0.142904), correlation (0.446114)*/,
	-12,-9, -6,4/*mean (0.146165), correlation (0.451473)*/,
	3,7, 7,12/*mean (0.147627), correlation (0.456643)*/,
	5,5, 10,8/*mean (0.152901), correlation (0.455036)*/,
	0,-4, 2,8/*mean (0.167083), correlation (0.459315)*/,
	-9,12, -5,-13/*mean (0.173234), correlation (0.454706)*/,
	0,7, 2,12/*mean (0.18312), correlation (0.433855)*/,
	-1,2, 1,7/*mean (0.185504), correlation (0.443838)*/,
	5,11, 7,-9/*mean (0.185706), correlation (0.451123)*/,
	3,5, 6,-8/*mean (0.188968), correlation (0.455808)*/,
	-13,-4, -8,9/*mean (0.191667), correlation (0.459128)*/,
	-5,9, -3,-3/*mean (0.193196), correlation (0.458364)*/,
	-4,-7, -3,-12/*mean (0.196536), correlation (0.455782)*/,
	6,5, 8,0/*mean (0.1972), correlation (0.450481)*/,
	-7,6, -6,12/*mean (0.199438), correlation (0.458156)*/,
	-13,6, -5,-2/*mean (0.211224), correlation (0.449548)*/,
	1,-10, 3,10/*mean (0.211718), correlation (0.440606)*/,
	4,1, 8,-4/*mean (0.213034), correlation (0.443177)*/,
	-2,-2, 2,-13/*mean (0.234334), correlation (0.455304)*/,
	2,-12, 12,12/*mean (0.235684), correlation (0.443436)*/,
	-2,-13, 0,-6/*mean (0.237674), correlation (0.452525)*/,
	4,1, 9,3/*mean (0.23962), correlation (0.444824)*/,
	-6,-10, -3,-5/*mean (0.248459), correlation (0.439621)*/,
	-3,-13, -1,1/*mean (0.249505), correlation (0.456666)*/,
	7,5, 12,-11/*mean (0.00119208), correlation (0.495466)*/,
	4,-2, 5,-7/*mean (0.00372245), correlation (0.484214)*/,
	-13,9, -9,-5/*mean (0.00741116), correlation (0.499854)*/,
	7,1, 8,6/*mean (0.0208952), correlation (0.499773)*/,
	7,-8, 7,6/*mean (0.0220085), correlation (0.501609)*/,
	-7,-4, -7,1/*mean (0.0233806), correlation (0.496568)*/,
	-8,11, -7,-8/*mean (0.0236505), correlation (0.489719)*/,
	-13,6, -12,-8/*mean (0.0268781), correlation (0.503487)*/,
	2,4, 3,9/*mean (0.0323324), correlation (0.501938)*/,
	10,-5, 12,3/*mean (0.0399235), correlation (0.494029)*/,
	-6,-5, -6,7/*mean (0.0420153), correlation (0.486579)*/,
	8,-3, 9,-8/*mean (0.0548021), correlation (0.484237)*/,
	2,-12, 2,8/*mean (0.0616622), correlation (0.496642)*/,
	-11,-2, -10,3/*mean (0.0627755), correlation (0.498563)*/,
	-12,-13, -7,-9/*mean (0.0829622), correlation (0.495491)*/,
	-11,0, -10,-5/*mean (0.0843342), correlation (0.487146)*/,
	5,-3, 11,8/*mean (0.0929937), correlation (0.502315)*/,
	-2,-13, -1,12/*mean (0.113327), correlation (0.48941)*/,
	-1,-8, 0,9/*mean (0.132119), correlation (0.467268)*/,
	-13,-11, -12,-5/*mean (0.136269), correlation (0.498771)*/,
	-10,-2, -10,11/*mean (0.142173), correlation (0.498714)*/,
	-3,9, -2,-13/*mean (0.144141), correlation (0.491973)*/,
	2,-3, 3,2/*mean (0.14892), correlation (0.500782)*/,
	-9,-13, -4,0/*mean (0.150371), correlation (0.498211)*/,
	-4,6, -3,-10/*mean (0.152159), correlation (0.495547)*/,
	-4,12, -2,-7/*mean (0.156152), correlation (0.496925)*/,
	-6,-11, -4,9/*mean (0.15749), correlation (0.499222)*/,
	6,-3, 6,11/*mean (0.159211), correlation (0.503821)*/,
	-13,11, -5,5/*mean (0.162427), correlation (0.501907)*/,
	11,11, 12,6/*mean (0.16652), correlation (0.497632)*/,
	7,-5, 12,-2/*mean (0.169141), correlation (0.484474)*/,
	-1,12, 0,7/*mean (0.169456), correlation (0.495339)*/,
	-4,-8, -3,-2/*mean (0.171457), correlation (0.487251)*/,
	-7,1, -6,7/*mean (0.175), correlation (0.500024)*/,
	-13,-12, -8,-13/*mean (0.175866), correlation (0.497523)*/,
	-7,-2, -6,-8/*mean (0.178273), correlation (0.501854)*/,
	-8,5, -6,-9/*mean (0.181107), correlation (0.494888)*/,
	-5,-1, -4,5/*mean (0.190227), correlation (0.482557)*/,
	-13,7, -8,10/*mean (0.196739), correlation (0.496503)*/,
	1,5, 5,-13/*mean (0.19973), correlation (0.499759)*/,
	1,0, 10,-13/*mean (0.204465), correlation (0.49873)*/,
	9,12, 10,-1/*mean (0.209334), correlation (0.49063)*/,
	5,-8, 10,-9/*mean (0.211134), correlation (0.503011)*/,
	-1,11, 1,-13/*mean (0.212), correlation (0.499414)*/,
	-9,-3, -6,2/*mean (0.212168), correlation (0.480739)*/,
	-1,-10, 1,12/*mean (0.212731), correlation (0.502523)*/,
	-13,1, -8,-10/*mean (0.21327), correlation (0.489786)*/,
	8,-11, 10,-6/*mean (0.214159), correlation (0.488246)*/,
	2,-13, 3,-6/*mean (0.216993), correlation (0.50287)*/,
	7,-13, 12,-9/*mean (0.223639), correlation (0.470502)*/,
	-10,-10, -5,-7/*mean (0.224089), correlation (0.500852)*/,
	-10,-8, -8,-13/*mean (0.228666), correlation (0.502629)*/,
	4,-6, 8,5/*mean (0.22906), correlation (0.498305)*/,
	3,12, 8,-13/*mean (0.233378), correlation (0.503825)*/,
	-4,2, -3,-3/*mean (0.234323), correlation (0.476692)*/,
	5,-13, 10,-12/*mean (0.236392), correlation (0.475462)*/,
	4,-13, 5,-1/*mean (0.236842), correlation (0.504132)*/,
	-9,9, -4,3/*mean (0.236977), correlation (0.497739)*/,
	0,3, 3,-9/*mean (0.24314), correlation (0.499398)*/,
	-12,1, -6,1/*mean (0.243297), correlation (0.489447)*/,
	3,2, 4,-8/*mean (0.00155196), correlation (0.553496)*/,
	-10,-10, -10,9/*mean (0.00239541), correlation (0.54297)*/,
	8,-13, 12,12/*mean (0.0034413), correlation (0.544361)*/,
	-8,-12, -6,-5/*mean (0.003565), correlation (0.551225)*/,
	2,2, 3,7/*mean (0.00835583), correlation (0.55285)*/,
	10,6, 11,-8/*mean (0.00885065), correlation (0.540913)*/,
	6,8, 8,-12/*mean (0.0101552), correlation (0.551085)*/,
	-7,10, -6,5/*mean (0.0102227), correlation (0.533635)*/,
	-3,-9, -3,9/*mean (0.0110211), correlation (0.543121)*/,
	-1,-13, -1,5/*mean (0.0113473), correlation (0.550173)*/,
	-3,-7, -3,4/*mean (0.0140913), correlation (0.554774)*/,
	-8,-2, -8,3/*mean (0.017049), correlation (0.55461)*/,
	4,2, 12,12/*mean (0.01778), correlation (0.546921)*/,
	2,-5, 3,11/*mean (0.0224022), correlation (0.549667)*/,
	6,-9, 11,-13/*mean (0.029161), correlation (0.546295)*/,
	3,-1, 7,12/*mean (0.0303081), correlation (0.548599)*/,
	11,-1, 12,4/*mean (0.0355151), correlation (0.523943)*/,
	-3,0, -3,6/*mean (0.0417904), correlation (0.543395)*/,
	4,-11, 4,12/*mean (0.0487292), correlation (0.542818)*/,
	2,-4, 2,1/*mean (0.0575124), correlation (0.554888)*/,
	-10,-6, -8,1/*mean (0.0594242), correlation (0.544026)*/,
	-13,7, -11,1/*mean (0.0597391), correlation (0.550524)*/,
	-13,12, -11,-13/*mean (0.0608974), correlation (0.55383)*/,
	6,0, 11,-13/*mean (0.065126), correlation (0.552006)*/,
	0,-1, 1,4/*mean (0.074224), correlation (0.546372)*/,
	-13,3, -9,-2/*mean (0.0808592), correlation (0.554875)*/,
	-9,8, -6,-3/*mean (0.0883378), correlation (0.551178)*/,
	-13,-6, -8,-2/*mean (0.0901035), correlation (0.548446)*/,
	5,-9, 8,10/*mean (0.0949843), correlation (0.554694)*/,
	2,7, 3,-9/*mean (0.0994152), correlation (0.550979)*/,
	-1,-6, -1,-1/*mean (0.10045), correlation (0.552714)*/,
	9,5, 11,-2/*mean (0.100686), correlation (0.552594)*/,
	11,-3, 12,-8/*mean (0.101091), correlation (0.532394)*/,
	3,0, 3,5/*mean (0.101147), correlation (0.525576)*/,
	-1,4, 0,10/*mean (0.105263), correlation (0.531498)*/,
	3,-6, 4,5/*mean (0.110785), correlation (0.540491)*/,
	-13,0, -10,5/*mean (0.112798), correlation (0.536582)*/,
	5,8, 12,11/*mean (0.114181), correlation (0.555793)*/,
	8,9, 9,-6/*mean (0.117431), correlation (0.553763)*/,
	7,-4, 8,-12/*mean (0.118522), correlation (0.553452)*/,
	-10,4, -10,9/*mean (0.12094), correlation (0.554785)*/,
	7,3, 12,4/*mean (0.122582), correlation (0.555825)*/,
	9,-7, 10,-2/*mean (0.124978), correlation (0.549846)*/,
	7,0, 12,-2/*mean (0.127002), correlation (0.537452)*/,
	-1,-6, 0,-11/*mean (0.127148), correlation (0.547401)*/
};

FeatureNode::FeatureNode() : only_one_(false)
{
}

void FeatureNode::DivideNode(FeatureNode(&p_nodes)[4])
{
	const int32_t half_x = (int32_t)ceil((float)(upper_right_.x - upper_left_.x) / 2);
	const int32_t half_y = (int32_t)ceil((float)(bottom_right_.y - upper_left_.y) / 2);
	const size_t keypoints_size = keypoints_.size();

	p_nodes[0].upper_left_ = upper_left_;
	p_nodes[0].upper_right_ = cv::Point2i(upper_left_.x + half_x, upper_left_.y);
	p_nodes[0].bottom_left_ = cv::Point2i(upper_left_.x, upper_left_.y + half_y);
	p_nodes[0].bottom_right_ = cv::Point2i(upper_left_.x + half_x, upper_left_.y + half_y);
	p_nodes[0].keypoints_.reserve(keypoints_size);

	p_nodes[1].upper_left_ = p_nodes[0].upper_right_;
	p_nodes[1].upper_right_ = upper_right_;
	p_nodes[1].bottom_left_ = p_nodes[0].bottom_right_;
	p_nodes[1].bottom_right_ = cv::Point2i(upper_right_.x, upper_left_.y + half_y);
	p_nodes[1].keypoints_.reserve(keypoints_size);

	p_nodes[2].upper_left_ = p_nodes[0].bottom_left_;
	p_nodes[2].upper_right_ = p_nodes[0].bottom_right_;
	p_nodes[2].bottom_left_ = bottom_left_;
	p_nodes[2].bottom_right_ = cv::Point2i(p_nodes[0].bottom_right_.x, bottom_left_.y);
	p_nodes[2].keypoints_.reserve(keypoints_size);

	p_nodes[3].upper_left_ = p_nodes[2].upper_right_;
	p_nodes[3].upper_right_ = p_nodes[1].bottom_right_;
	p_nodes[3].bottom_left_ = p_nodes[2].bottom_right_;
	p_nodes[3].bottom_right_ = bottom_right_;
	p_nodes[3].keypoints_.reserve(keypoints_size);

	for (size_t i = 0; i < keypoints_size; ++i)
	{
		if (keypoints_[i].pt.x < p_nodes[0].upper_right_.x)
		{
			if (keypoints_[i].pt.y < p_nodes[0].bottom_right_.y)
			{
				p_nodes[0].keypoints_.push_back(keypoints_[i]);
			}
			else
			{
				p_nodes[2].keypoints_.push_back(keypoints_[i]);
			}
		}
		else
		{
			if (keypoints_[i].pt.y < p_nodes[0].bottom_right_.y)
			{
				p_nodes[1].keypoints_.push_back(keypoints_[i]);
			}
			else
			{
				p_nodes[3].keypoints_.push_back(keypoints_[i]);
			}
		}
	}

	for (int32_t i = 0; i < 4; ++i)
	{
		p_nodes[i].only_one_ = (p_nodes[i].keypoints_.size() == 1) ? true : false;
	}
}

ORBFeature::ORBFeature(const int32_t p_features_max, const float p_scale_factor, const int32_t p_levels, const int32_t p_fast_threshold_init, const int32_t p_fast_threshold_min) :
	features_max_(p_features_max), scale_factor_(p_scale_factor), levels_(p_levels),
	fast_threshold_init_(p_fast_threshold_init), fast_threshold_min_(p_fast_threshold_min),
	kPatchSize_(31), kHalfPatchSize_(15), kEdgeThreshold_(19)
{
	scale_factor_vector_.resize(levels_);
	scale_factor_vector_[0] = 1.0f;
	for (int32_t i = 1; i < levels_; ++i)
	{
		scale_factor_vector_[i] = scale_factor_vector_[i - 1] * scale_factor_;
	}

	image_pyramid_.resize(levels_);

	features_per_level_.resize(levels_);
	float factor = 1.0f / (float)scale_factor_;
	float features_per_level = features_max_ * (1 - factor) / (1 - (float)pow((double)factor, (double)levels_));

	int32_t features_sum = 0;
	for (int32_t level = 0; level < levels_ - 1; ++level)
	{
		features_per_level_[level] = cvRound(features_per_level);
		features_sum += features_per_level_[level];
		features_per_level *= factor;
	}
	features_per_level_[levels_ - 1] = std::max(features_max_ - features_sum, 0);

	const cv::Point * pattern = (const cv::Point *)kPattern;
	std::copy(pattern, pattern + 512, std::back_inserter(pattern_));

	u_max_.resize(kHalfPatchSize_ + 1);

	const int32_t v_max = (int32_t)floor(kHalfPatchSize_ * sqrt(2.f) / 2 + 1);
	const int32_t v_min = (int32_t)ceil(kHalfPatchSize_ * sqrt(2.f) / 2);
	const double half_patch_square = kHalfPatchSize_ * kHalfPatchSize_;

	for (int32_t v = 0; v <= v_max; ++v)
	{
		u_max_[v] = (int32_t)round(sqrt(half_patch_square - v * v));
	}

	for (int32_t v = kHalfPatchSize_, v0 = 0; v >= v_min; --v, ++v0)
	{
		while (u_max_[v0] == u_max_[v0 + 1])
		{
			++v0;
		}
		u_max_[v] = v0;
	}
}

void ORBFeature::ComputeOrientation(const cv::Mat & p_image, std::vector<cv::KeyPoint> & p_keypoints)
{
	const int32_t step = (int32_t)p_image.step1();
	const int32_t keypoints_size = (int32_t)p_keypoints.size();

	#pragma omp parallel for
	for (int32_t i = 0; i < keypoints_size; ++i)
	{
		int32_t m_01 = 0, m_10 = 0;
		const uint8_t * center = &p_image.at<uint8_t>((int32_t)round(p_keypoints[i].pt.y), (int32_t)round(p_keypoints[i].pt.x));

		for (int32_t u = -kHalfPatchSize_; u <= kHalfPatchSize_; ++u)
		{
			m_10 += u * center[u];
		}

		for (int32_t v = 1; v <= kHalfPatchSize_; ++v)
		{
			int32_t v_sum = 0;
			int32_t edge = u_max_[v];
			for (int32_t u = -edge; u <= edge; ++u)
			{
				int32_t value_plus = center[u + v * step];
				int32_t value_minus = center[u - v * step];
				v_sum += (value_plus - value_minus);
				m_10 += u * (value_plus + value_minus);
			}
			m_01 += v * v_sum;
		}

		p_keypoints[i].angle = cv::fastAtan2((float)m_01, (float)m_10);
	}
}

std::vector<cv::KeyPoint> ORBFeature::DistributeTree(const std::vector<cv::KeyPoint>& p_distributed_keypoints, const int32_t & p_x_min,
	const int32_t & p_x_max, const int32_t & p_y_min, const int32_t & p_y_max, const int32_t & p_features_number)
{
	int32_t nodes_init_number = (int32_t)round((float)(p_x_max - p_x_min) / (p_y_max - p_y_min));
	float nodes_interval = (float)(p_x_max - p_x_min) / nodes_init_number;

	std::list<FeatureNode> nodes_list;
	std::vector<FeatureNode *> nodes_init;
	nodes_init.resize(nodes_init_number);

	for (int32_t i = 0; i < nodes_init_number; ++i)
	{
		FeatureNode feature_node;
		feature_node.upper_left_ = cv::Point2i((int32_t)(nodes_interval * (float)(i)), 0);
		feature_node.upper_right_ = cv::Point2i((int32_t)(nodes_interval * (float)(i + 1)), 0);
		feature_node.bottom_left_ = cv::Point2i(feature_node.upper_left_.x, p_y_max - p_y_min);
		feature_node.bottom_right_ = cv::Point2i(feature_node.upper_right_.x, p_y_max - p_y_min);
		feature_node.keypoints_.reserve(p_distributed_keypoints.size());

		nodes_list.push_back(feature_node);
		nodes_init[i] = &nodes_list.back();
	}

	for (size_t i = 0, for_size = p_distributed_keypoints.size(); i < for_size; ++i)
	{
		nodes_init[(size_t)(p_distributed_keypoints[i].pt.x / nodes_interval)]->keypoints_.push_back(p_distributed_keypoints[i]);
	}

	for (std::list<FeatureNode>::iterator list_iterator = nodes_list.begin(), list_end = nodes_list.end(); list_iterator != list_end;)
	{
		if (list_iterator->keypoints_.empty())
		{
			list_iterator = nodes_list.erase(list_iterator);
		}
		else
		{
			list_iterator->only_one_ = (list_iterator->keypoints_.size() == 1) ? true : false;
			++list_iterator;
		}
	}

	std::vector<std::pair<int32_t, FeatureNode *> > nodes_size_and_pointer;
	nodes_size_and_pointer.reserve(nodes_list.size() * 4);

	for (bool is_finish = false; !is_finish;)
	{
		int32_t pre_nodes_list_size = (int32_t)nodes_list.size();
		int32_t to_expand = 0;

		nodes_size_and_pointer.clear();

		for (std::list<FeatureNode>::iterator list_iterator = nodes_list.begin(), list_end = nodes_list.end(); list_iterator != list_end;)
		{
			if (list_iterator->only_one_)
			{
				++list_iterator;
			}
			else
			{
				FeatureNode feature_nodes[4];
				list_iterator->DivideNode(feature_nodes);

				for (int32_t nodes_index = 0; nodes_index < 4; ++nodes_index)
				{
					int32_t nodes_keypoints_size = (int32_t)feature_nodes[nodes_index].keypoints_.size();
					if (nodes_keypoints_size > 0)
					{
						nodes_list.push_front(feature_nodes[nodes_index]);
						if (nodes_keypoints_size > 1)
						{
							++to_expand;
							nodes_size_and_pointer.push_back(std::make_pair(nodes_keypoints_size, &nodes_list.front()));
							nodes_list.front().iterator_ = nodes_list.begin();
						}
					}
				}

				list_iterator = nodes_list.erase(list_iterator);
			}
		}

		int32_t cur_nodes_list_size = (int32_t)nodes_list.size();
		if ((cur_nodes_list_size >= p_features_number) || (cur_nodes_list_size == pre_nodes_list_size))
		{
			is_finish = true;
		}
		else if ((cur_nodes_list_size + to_expand * 3) > p_features_number)
		{
			while (!is_finish)
			{
				pre_nodes_list_size = (int32_t)nodes_list.size();

				std::vector<std::pair<int32_t, FeatureNode *> > pre_nodes_size_and_pointer = nodes_size_and_pointer;
				nodes_size_and_pointer.clear();
				std::sort(pre_nodes_size_and_pointer.begin(), pre_nodes_size_and_pointer.end());

				for (int32_t j = (int32_t)pre_nodes_size_and_pointer.size() - 1; j >= 0; --j)
				{
					FeatureNode feature_nodes[4];
					pre_nodes_size_and_pointer[j].second->DivideNode(feature_nodes);

					for (int32_t nodes_index = 0; nodes_index < 4; ++nodes_index)
					{
						int32_t nodes_keypoints_size = (int32_t)feature_nodes[nodes_index].keypoints_.size();
						if (nodes_keypoints_size > 0)
						{
							nodes_list.push_front(feature_nodes[nodes_index]);
							if (nodes_keypoints_size > 1)
							{
								nodes_size_and_pointer.push_back(std::make_pair(nodes_keypoints_size, &nodes_list.front()));
								nodes_list.front().iterator_ = nodes_list.begin();
							}
						}
					}

					nodes_list.erase(pre_nodes_size_and_pointer[j].second->iterator_);

					if ((int32_t)nodes_list.size() >= p_features_number)
					{
						break;
					}
				}

				cur_nodes_list_size = (int32_t)nodes_list.size();
				if ((cur_nodes_list_size >= p_features_number) || (cur_nodes_list_size == pre_nodes_list_size))
				{
					is_finish = true;
				}
			}
		}
	}

	std::vector<cv::KeyPoint> keypoints_result;
	keypoints_result.reserve(features_max_);

	for (auto &list_node : nodes_list)
	{
		float response_max = list_node.keypoints_[0].response;
		size_t response_max_index = 0;
		size_t keypoints_size = list_node.keypoints_.size();
		for (size_t i = 1; i < keypoints_size; ++i)
		{
			if (list_node.keypoints_[i].response > response_max)
			{
				response_max_index = i;
				response_max = list_node.keypoints_[i].response;
			}
		}

		keypoints_result.push_back(list_node.keypoints_[response_max_index]);
	}

	return keypoints_result;
}

void ORBFeature::ComputeKeyPoints(std::vector<std::vector<cv::KeyPoint> > & p_keypoints)
{
	p_keypoints.resize(levels_);

	for (int32_t level = 0; level < levels_; ++level)
	{
		const int32_t border_x_min = kEdgeThreshold_ - 3;
		const int32_t border_y_min = kEdgeThreshold_ - 3;
		const int32_t border_x_max = image_pyramid_[level].cols - (kEdgeThreshold_ - 3);
		const int32_t border_y_max = image_pyramid_[level].rows - (kEdgeThreshold_ - 3);

		const float width = (float)(border_x_max - border_x_min);
		const float height = (float)(border_y_max - border_y_min);
		const int32_t columns = (int32_t)(width / 30);
		const int32_t rows = (int32_t)(height / 30);
		const int32_t width_unit = (int32_t)ceil(width / columns);
		const int32_t height_unit = (int32_t)ceil(height / rows);

		std::vector<cv::KeyPoint> distributed_keypoints;
		distributed_keypoints.reserve(features_max_ * 10);

		for (int32_t i = 0; i < rows; ++i)
		{
			float y_start = (float)(border_y_min + i * height_unit);
			if (y_start >= (border_y_max - 3))
			{
				continue;
			}

			float y_end = y_start + height_unit + 6;
			if (y_end > border_y_max)
			{
				y_end = (float)border_y_max;
			}

			for (int32_t j = 0; j < columns; ++j)
			{
				float x_start = (float)(border_x_min + j * width_unit);
				if (x_start >= (border_x_max - 6))
				{
					continue;
				}

				float x_end = x_start + width_unit + 6;
				if (x_end > border_x_max)
				{
					x_end = (float)border_x_max;
				}

				std::vector<cv::KeyPoint> keypoints_of_point;
				cv::FAST(image_pyramid_[level].rowRange((int32_t)y_start, (int32_t)y_end).colRange((int32_t)x_start, (int32_t)x_end),
					keypoints_of_point, fast_threshold_init_, true);

				if (keypoints_of_point.empty())
				{
					cv::FAST(image_pyramid_[level].rowRange((int32_t)y_start, (int32_t)y_end).colRange((int32_t)x_start, (int32_t)x_end),
						keypoints_of_point, fast_threshold_min_, true);
				}

				if (!keypoints_of_point.empty())
				{
					int32_t keypoints_of_point_size = (int32_t)keypoints_of_point.size();
					for (int32_t k = 0; k < keypoints_of_point_size; ++k)
					{
						keypoints_of_point[k].pt.x += j * width_unit;
						keypoints_of_point[k].pt.y += i * height_unit;
						distributed_keypoints.push_back(keypoints_of_point[k]);
					}
				}

			}
		}

		p_keypoints[level].reserve(features_max_);
		p_keypoints[level] = DistributeTree(distributed_keypoints, border_x_min, border_x_max,
			border_y_min, border_y_max, features_per_level_[level]);

		const float scale_patch_size = (float)(kPatchSize_ * scale_factor_vector_[level]);
		const int32_t keypoints_level_size = (int32_t)p_keypoints[level].size();

		#pragma omp parallel for
		for (int32_t i = 0; i < keypoints_level_size; ++i)
		{
			p_keypoints[level][i].pt.x += border_x_min;
			p_keypoints[level][i].pt.y += border_y_min;
			p_keypoints[level][i].octave = level;
			p_keypoints[level][i].size = scale_patch_size;
		}
	}

	#pragma omp parallel for
	for (int32_t level = 0; level < levels_; ++level)
	{
		ComputeOrientation(image_pyramid_[level], p_keypoints[level]);
	}
}

void ORBFeature::ComputeDescriptors(const cv::Mat & p_image, const std::vector<cv::KeyPoint> & p_keypoints, cv::Mat & p_descriptors)
{
	const int32_t keypoints_size = (int32_t)p_keypoints.size();
	const int32_t step = (int32_t)p_image.step;
	const float to_pi = (float)(CV_PI / 180.f);
	p_descriptors = cv::Mat::zeros(keypoints_size, 32, CV_8UC1);

	#pragma omp parallel for
	for (int32_t count = 0; count < keypoints_size; ++count)
	{
		float angle = (float)p_keypoints[count].angle * to_pi;
		float angle_cos = (float)cos(angle);
		float angle_sin = (float)sin(angle);

		const uint8_t * center = &p_image.at<uint8_t>(cvRound(p_keypoints[count].pt.y), cvRound(p_keypoints[count].pt.x));
		for (int32_t i = 0, j = 0; i < 32; ++i, j += 16)
		{
			int32_t t0, t1, pattern_index;
			int32_t bit_value = 0;
			for (int32_t k = 0; k < 16; k += 2)
			{
				pattern_index = j + k;
				t0 = center[(int32_t)round(pattern_[pattern_index].x * angle_sin + pattern_[pattern_index].y * angle_cos) * step +
					(int32_t)round(pattern_[pattern_index].x * angle_cos - pattern_[pattern_index].y * angle_sin)];
				t1 = center[(int32_t)round(pattern_[pattern_index + 1].x * angle_sin + pattern_[pattern_index + 1].y * angle_cos) * step +
					(int32_t)round(pattern_[pattern_index + 1].x * angle_cos - pattern_[pattern_index + 1].y * angle_sin)];
				bit_value |= (t0 < t1) << (k / 2);
			}
			p_descriptors.ptr(count)[i] = (uint8_t)bit_value;
		}
	}
}

void ORBFeature::operator()(cv::InputArray p_image, std::vector<cv::KeyPoint> & p_keypoints, cv::OutputArray p_descriptors)
{
	if (p_image.empty()) return;
	cv::Mat image = p_image.getMat();
	assert(image.type() == CV_8UC1);

	ComputePyramid(image);

	std::vector<std::vector<cv::KeyPoint> > keypoints;
	ComputeKeyPoints(keypoints);

	cv::Mat descriptors;

	int32_t keypoints_sum = 0;
	for (int32_t level = 0; level < levels_; ++level)
	{
		keypoints_sum += (int32_t)keypoints[level].size();
	}

	if (keypoints_sum == 0)
	{
		p_descriptors.release();
	}
	else
	{
		p_descriptors.create(keypoints_sum, 32, CV_8U);
		descriptors = p_descriptors.getMat();
	}

	p_keypoints.clear();
	p_keypoints.reserve(keypoints_sum);

	for (int32_t level = 0, offset = 0; level < levels_; ++level)
	{
		int32_t keypoints_level_size = (int32_t)keypoints[level].size();
		if (keypoints_level_size == 0) continue;

		cv::Mat Gaussian_image = image_pyramid_[level].clone();
		cv::GaussianBlur(Gaussian_image, Gaussian_image, cv::Size(7, 7), 2.0, 2.0, cv::BORDER_REFLECT_101);

		cv::Mat descriptor = descriptors.rowRange(offset, offset + keypoints_level_size);
		ComputeDescriptors(Gaussian_image, keypoints[level], descriptor);

		offset += keypoints_level_size;

		if (level != 0)
		{
			float scale = scale_factor_vector_[level];
			for (int32_t i = 0; i < keypoints_level_size; ++i)
			{
				keypoints[level][i].pt *= scale;
			}
		}

		p_keypoints.insert(p_keypoints.end(), keypoints[level].begin(), keypoints[level].end());
	}
}

void ORBFeature::ComputePyramid(cv::Mat p_image)
{
	for (int32_t level = 0; level < levels_; ++level)
	{
		float scale = 1.0f / scale_factor_vector_[level];
		cv::Size size((int32_t)round((float)p_image.cols * scale), (int32_t)round((float)p_image.rows * scale));
		cv::Size whole_size(size.width + kEdgeThreshold_ * 2, size.height + kEdgeThreshold_ * 2);
		cv::Mat temp(whole_size, p_image.type());
		image_pyramid_[level] = temp(cv::Rect(kEdgeThreshold_, kEdgeThreshold_, size.width, size.height));

		if (level != 0)
		{
			resize(image_pyramid_[level - 1], image_pyramid_[level], size, 0, 0, cv::INTER_LINEAR);

			cv::copyMakeBorder(image_pyramid_[level], temp, kEdgeThreshold_, kEdgeThreshold_, kEdgeThreshold_, kEdgeThreshold_,
				cv::BORDER_REFLECT_101 + cv::BORDER_ISOLATED);
		}
		else
		{
			cv::copyMakeBorder(p_image, temp, kEdgeThreshold_, kEdgeThreshold_, kEdgeThreshold_, kEdgeThreshold_,
				cv::BORDER_REFLECT_101);
		}
	}

}
