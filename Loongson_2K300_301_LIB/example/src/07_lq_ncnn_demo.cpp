#include "lq_all_demo.hpp"
#include <opencv2/opencv.hpp>

// 获取当前时间戳字符串
static string GetTimestamp()
{
    auto now = chrono::system_clock::now();
    auto ms = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()) % 1000;
    time_t t = chrono::system_clock::to_time_t(now);
    tm* tm = localtime(&t);

    stringstream ss;
    ss << put_time(tm, "%Y-%m-%d %H:%M:%S") << "." << setfill('0') << setw(3) << ms.count();
    return ss.str();
}

void lq_ncnn_demo(void)
{
    try {
        std::cout << "========== NCNN 环境自检(无模型) ==========" << std::endl;

        // 创建 ncnn::Net
        ncnn::Net net;
        net.opt.num_threads = 2;
        net.opt.use_vulkan_compute = false;  // 基础检查

        // 创建测试输入张量，验证 Mat 内存与基础操作
        ncnn::Mat input(8, 8, 3); // w, h, c
        for (int c = 0; c < input.c; ++c) {
            float* ptr = input.channel(c);
            for (int i = 0; i < input.w * input.h; ++i) {
                ptr[i] = static_cast<float>(c * 100 + i);
            }
        }

        ncnn::Mat input_clone = input.clone();
        float checksum = 0.0f;
        for (int c = 0; c < input_clone.c; ++c) {
            const float* ptr = input_clone.channel(c);
            for (int i = 0; i < input_clone.w * input_clone.h; ++i) {
                checksum += ptr[i];
            }
        }

        // 创建提取器
        ncnn::Extractor ex = net.create_extractor();

        // 未加载模型时，输入层不存在，返回非 0 属于预期行为
        int ret = ex.input("data", input_clone);

        std::cout << "Mat 维度: w=" << input_clone.w
                  << ", h=" << input_clone.h
                  << ", c=" << input_clone.c << std::endl;
        std::cout << "Mat 校验和: " << checksum << std::endl;
        std::cout << "Extractor input 返回值(无模型预期非0): " << ret << std::endl;

        if (ret != 0) {
            std::cout << "[PASS] ncnn 基础环境可用（头文件/链接/运行时对象均正常）。" << std::endl;
            return;
        }

        std::cout << "[WARN] 未加载模型却返回0，建议进一步检查 ncnn 版本与调用逻辑。" << std::endl;
        return;
    } catch (const std::exception& e) {
        std::cerr << "[FAIL] ncnn 环境自检异常: " << e.what() << std::endl;
        return;
    } catch (...) {
        std::cerr << "[FAIL] ncnn 环境自检出现未知异常" << std::endl;
        return;
    }
}
void lq_ncnn_photo_demo(void)
{
    printf("========================================\n");
    printf("       NCNN 图像分类推理示例\n");
    printf("========================================\n\n");

    // ==================== 配置区域 ====================
    // 测试图片路径（修改为你的图片路径）
    string test_image_path = "test.jpg";
    
    // 模型配置
    string model_param = "tiny_classifier_fp32.ncnn.param";
    string model_bin   = "tiny_classifier_fp32.ncnn.bin";
    int input_width    = 96;
    int input_height   = 96;
    
    // 类别标签（顺序必须与训练时一致）
    vector<string> labels = {"supplies", "vehicle", "weapon"};
    
    // 归一化参数（ImageNet标准）
    float mean_vals[3] = {123.675f, 116.28f, 103.53f};
    float norm_vals[3] = {0.01712475f, 0.017507f, 0.01742919f};
    // =================================================

    // 创建NCNN对象并配置
    LQ_NCNN ncnn;
    ncnn.SetModelPath(model_param, model_bin);
    ncnn.SetInputSize(input_width, input_height);
    ncnn.SetLabels(labels);
    ncnn.SetNormalize(mean_vals, norm_vals);

    // 初始化模型
    printf("[%s] 正在加载模型...\n", GetTimestamp().c_str());
    if (!ncnn.Init()) {
        printf("[%s] 模型加载失败!\n", GetTimestamp().c_str());
        return ;
    }
    printf("[%s] 模型加载成功!\n\n", GetTimestamp().c_str());

    // 读取测试图片
    printf("[%s] 读取图片: %s\n", GetTimestamp().c_str(), test_image_path.c_str());
    cv::Mat image = cv::imread(test_image_path);
    
    if (image.empty()) {
        printf("无法读取图片: %s\n", test_image_path.c_str());
        printf("请确保图片文件存在\n");
        return ;
    }
    printf("[%s] 图片尺寸: %d x %d\n\n", GetTimestamp().c_str(), image.cols, image.rows);

    // 推理
    printf("[%s] 开始推理...\n", GetTimestamp().c_str());
    auto start = chrono::high_resolution_clock::now();
    string result = ncnn.Infer(image);
    auto end = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::milliseconds>(end - start);

    // 输出结果
    printf("\n========================================\n");
    printf("推理结果: %s\n", result.c_str());
    printf("推理耗时: %ld ms\n", duration.count());
    printf("========================================\n");

    return ;
}

