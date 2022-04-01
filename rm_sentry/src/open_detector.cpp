#include "rm_sentry/open_detector.hpp"

namespace rm_sentry
{
    const int TOPK_NUM = 128;
    Detector::Detector(string xml_path, double cof_threshold)
    {
        // std::cout << "openvino init" << std::endl;
        this->init(xml_path, cof_threshold);
        // std::cout << "openvino init finish" << std::endl;
    }

    Detector::~Detector() {}

    //注意此处的阈值是框和物体prob乘积的阈值

    bool Detector::parse_yolov5(const Blob::Ptr &blob, int net_grid, float cof_threshold,
                                vector<Bbox> &raw_detected)
    {
        vector<int> anchors = get_anchors(net_grid);
        LockedMemory<const void> blobMapped = as<MemoryBlob>(blob)->rmap();
        const float *output_blob = blobMapped.as<float *>();

        // 80个类是85,一个类是6,n个类是n+5
        // int item_size = 6;
        int item_size = 45;
        size_t anchor_n = 5;

        for (size_t n = 0; n < anchor_n; ++n)
            for (int i = 0; i < net_grid; ++i)
                for (int j = 0; j < net_grid; ++j)
                {
                    double box_prob = output_blob[n * net_grid * net_grid * item_size + i * net_grid * item_size + j * item_size + 8];
                    box_prob = sigmoid(box_prob);
                    //框置信度不满足则整体置信度不满足
                    if (box_prob < cof_threshold)
                        continue;

                    vector<Point2f> coord(4);
                    vector<float> xx(4), yy(4);
                    for (int k = 0; k < 4; k++)
                    {
                        // cout << " ++++++++++++++++ " << endl;
                        xx[k] = output_blob[n * net_grid * net_grid * item_size + i * net_grid * item_size + j * item_size + 2 * k];
                        yy[k] = output_blob[n * net_grid * net_grid * item_size + i * net_grid * item_size + j * item_size + 2 * k + 1];
                    }
                    // cout << " *********** " << endl;
                    //注意此处输出为中心点坐标,需要转化为角点坐标
                    // double x1 = output_blob[n * net_grid * net_grid * item_size + i * net_grid * item_size + j * item_size + 0];
                    // double y1 = output_blob[n * net_grid * net_grid * item_size + i * net_grid * item_size + j * item_size + 1];
                    // double w = output_blob[n * net_grid * net_grid * item_size + i * net_grid * item_size + j * item_size + 2];
                    // double h = output_blob[n * net_grid * net_grid * item_size + i * net_grid * item_size + j * item_size + 3];

                    double max_prob = 0;
                    int idx = 0;
                    for (int t = 9; t < 45; ++t)
                    {
                        double tp = output_blob[n * net_grid * net_grid * item_size + i * net_grid * item_size + j * item_size + t];
                        // cout<<tp << endl;
                        
                        if (tp > max_prob)
                        {
                            max_prob = tp;
                            idx = t;
                        }
                    }
                    float cof = box_prob * max_prob;
                    //对于边框置信度小于阈值的边框,不关心其他数值,不进行计算减少计算量
                    if (cof < cof_threshold)
                        continue;
                    for (int k = 0; k < 4; k++)
                    {
                        xx[k] = (xx[k] + j) * 640 / net_grid;
                        yy[k] = (yy[k] + i) * 640 / net_grid;
                    }
                    // x1 = (sigmoid(x1) * 2 - 0.5 + j) * 640.0f / net_grid;
                    // y1 = (sigmoid(y1) * 2 - 0.5 + i) * 640.0f / net_grid;
                    // w = pow(sigmoid(w)*2,2) * anchors[n*2];
                    // h = pow(sigmoid(h)*2,2) * anchors[n*2 + 1];

                    // double r_x = x1 - w/2;
                    // double r_y = y1 - h/2;
                    for (int k = 0; k < 4; k++)
                    {
                        coord[k] = {xx[k], yy[k]};
                    }
                    // o_rect_cof.push_back(cof);
                    // coords.push_back(coord);
                    int color_id = idx / 9;
                    int tag_id = idx % 9;
                    raw_detected.push_back({idx, color_id, tag_id, cof, coord});
                }
        if (raw_detected.size() == 0)
            return false;
        else
            return true;
    }

    //初始化
    bool Detector::init(string xml_path, double cof_threshold)
    {
        _xml_path = xml_path;
        _cof_threshold = cof_threshold;
        Core ie;
        auto cnnNetwork = ie.ReadNetwork(_xml_path);
        //输入设置
        InputsDataMap inputInfo(cnnNetwork.getInputsInfo());
        InputInfo::Ptr &input = inputInfo.begin()->second;
        _input_name = inputInfo.begin()->first;
        input->setPrecision(Precision::FP32);
        input->getInputData()->setLayout(Layout::NCHW);
        ICNNNetwork::InputShapes inputShapes = cnnNetwork.getInputShapes();
        SizeVector &inSizeVector = inputShapes.begin()->second;
        cnnNetwork.reshape(inputShapes);
        //输出设置
        _outputinfo = OutputsDataMap(cnnNetwork.getOutputsInfo());
        for (auto &output : _outputinfo)
        {
            output.second->setPrecision(Precision::FP32);
        }
        //获取可执行网络
        _network = ie.LoadNetwork(cnnNetwork, "GPU");
        //_network = ie.LoadNetwork(cnnNetwork, "CPU");
        return true;
    }

    //释放资源
    bool Detector::uninit()
    {
        return true;
    }

    //处理图像获取结果
    bool Detector::process_frame(Mat &inframe, vector<Bbox> &detected_objects)
    {
        // std::cout << "openvino process" << std::endl;
        if (inframe.empty())
        {
            cout << "无效图片输入" << endl;
            return false;
        }
        resize(inframe, inframe, Size(640, 640));
        // cvtColor(inframe, inframe, COLOR_BGR2RGB);
        size_t img_size = 640 * 640;
        InferRequest::Ptr infer_request = _network.CreateInferRequestPtr();
        Blob::Ptr frameBlob = infer_request->GetBlob(_input_name);
        InferenceEngine::LockedMemory<void> blobMapped = InferenceEngine::as<InferenceEngine::MemoryBlob>(frameBlob)->wmap();
        float *blob_data = blobMapped.as<float *>();
        // nchw
        for (size_t row = 0; row < 640; row++)
        {
            for (size_t col = 0; col < 640; col++)
            {
                for (size_t ch = 0; ch < 3; ch++)
                {
                    blob_data[img_size * ch + row * 640 + col] = float(inframe.at<Vec3b>(row, col)[ch]) / 255.0f;
                }
            }
        }
        //执行预测
        infer_request->Infer();

        //获取各层结果
        // vector<Rect> origin_rect;
        // vector<float> origin_rect_cof;
        // vector<vector<pair<double,double>>> origin_coord;
        vector<Bbox> raw_detected;
        int s[3] = {80, 40, 20};
        int i = 0;
        for (auto &output : _outputinfo)
        {

            auto output_name = output.first;
            // cout << output_name << endl;
            Blob::Ptr blob = infer_request->GetBlob(output_name);

            parse_yolov5(blob, s[i], _cof_threshold, raw_detected);
            ++i;
            if (i == 3)
                break;
        }
        //后处理获得最终检测结果
        sort(raw_detected.begin(), raw_detected.end(), [](Bbox a, Bbox b) -> bool
             { return a.cof > b.cof; });
        bool removed[TOPK_NUM];
        for (size_t k = 0; k < TOPK_NUM; k++)
        {
            if (k >= raw_detected.size())
                break;
            if (removed[k])
                continue;
            for (size_t m = k + 1; m < TOPK_NUM; m++)
            {
                if (m >= raw_detected.size())
                    break;
                if (removed[m])
                    continue;
                if (is_overlap(raw_detected[k].pts, raw_detected[m].pts))
                    removed[m] = true;
            }
            detected_objects.push_back(raw_detected[k]);
        }
        vector<int> final_id;
        // dnn::NMSBoxes(origin_rect,origin_rect_cof,_cof_threshold,_nms_area_threshold,final_id);
        //根据final_id获取最终结果
        //  for(int i=0;i<final_id.size();++i){
        //      Rect resize_rect= origin_rect[final_id[i]];
        //      detected_objects.push_back(Object{
        //          origin_coord,
        //          "",resize_rect
        //      });
        //  }
        return true;
    }

    //以下为工具函数
    double Detector::sigmoid(double x)
    {
        return (1 / (1 + exp(-x)));
    }

    vector<int> Detector::get_anchors(int net_grid)
    {
        vector<int> anchors(6);
        int a80[6] = {10, 13, 16, 30, 33, 23};
        int a40[6] = {30, 61, 62, 45, 59, 119};
        int a20[6] = {116, 90, 156, 198, 373, 326};
        if (net_grid == 80)
        {
            anchors.insert(anchors.begin(), a80, a80 + 6);
        }
        else if (net_grid == 40)
        {
            anchors.insert(anchors.begin(), a40, a40 + 6);
        }
        else if (net_grid == 20)
        {
            anchors.insert(anchors.begin(), a20, a20 + 6);
        }
        return anchors;
    }

    // template<class T, class ...Ts>
    // T reduce_max(T x, Ts... xs) {
    //     return reduce([](auto &&a, auto &&b){return std::max(a, b);}, x, xs...);
    // }

    // template<class T, class ...Ts>
    // T reduce_min(T x, Ts... xs) {
    //     return reduce([](auto &&a, auto &&b){return std::min(a, b);}, x, xs...);
    // }

    double get_max(double b1, double b2, double b3, double b4)
    {
        double max_value = b1;
        max_value = std::max(max_value, b2);
        max_value = std::max(max_value, b3);
        max_value = std::max(max_value, b4);
        return max_value;
    }

    double get_min(double b1, double b2, double b3, double b4)
    {
        double min_value = b1;
        min_value = std::min(min_value, b2);
        min_value = std::min(min_value, b3);
        min_value = std::min(min_value, b4);
        return min_value;
    }
    bool Detector::is_overlap(vector<Point2f> pts1, vector<Point2f> pts2)
    {
        cv::Rect2f bbox1, bbox2;
        bbox1.x = get_min(pts1[0].x, pts1[1].x, pts1[2].x, pts1[3].x);
        bbox1.y = get_min(pts1[0].y, pts1[1].y, pts1[2].y, pts1[3].y);
        bbox1.width = get_max(pts1[0].x, pts1[1].x, pts1[2].x, pts1[3].x) - bbox1.x;
        bbox1.height = get_max(pts1[0].y, pts1[1].y, pts1[2].y, pts1[3].y) - bbox1.y;
        bbox2.x = get_min(pts2[0].x, pts2[1].x, pts2[2].x, pts2[3].x);
        bbox2.y = get_min(pts2[0].y, pts2[1].y, pts2[2].y, pts2[3].y);
        bbox2.width = get_max(pts2[0].x, pts2[1].x, pts2[2].x, pts2[3].x) - bbox2.x;
        bbox2.height = get_max(pts2[0].y, pts2[1].y, pts2[2].y, pts2[3].y) - bbox2.y;
        return (bbox1 & bbox2).area() > 0;
    }
}