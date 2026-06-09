#include <dobot_bringup/command.h>
#include <cctype>
#include <iostream>
#include <chrono>
#include <thread>

namespace
{
std::string trimDashboardReply(std::string reply)
{
    while (!reply.empty())
    {
        const char ch = reply.back();
        if (ch == '\0' || ch == '\r' || ch == '\n' || ch == '\t')
        {
            reply.pop_back();
            continue;
        }
        break;
    }
    return reply;
}

std::string readDashboardReply(std::shared_ptr<TcpClient> &tcp)
{
    std::string reply;
    bool has_any_data = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);

    while (std::chrono::steady_clock::now() < deadline)
    {
        uint32_t has_read = 0;
        char chunk[1024];
        memset(chunk, 0, sizeof(chunk));

        const bool terminated = tcp->tcpRecv(chunk, sizeof(chunk) - 1, has_read, has_any_data ? 200 : 1000);
        if (has_read > 0)
        {
            reply.append(chunk, has_read);
            has_any_data = true;

            const char last = reply.back();
            if (last == ';' || last == '\r' || last == '\n' || last == '\t')
            {
                break;
            }
        }

        if (terminated || has_any_data)
        {
            break;
        }
    }

    if (!has_any_data)
    {
        throw TcpClientException("dashboard reply timeout");
    }

    return trimDashboardReply(reply);
}

int32_t parseDashboardErrorId(const std::string &reply)
{
    size_t pos = 0;
    while (pos < reply.size() && std::isspace(static_cast<unsigned char>(reply[pos])))
    {
        ++pos;
    }

    const size_t start = pos;
    if (pos < reply.size() && (reply[pos] == '+' || reply[pos] == '-'))
    {
        ++pos;
    }
    while (pos < reply.size() && std::isdigit(static_cast<unsigned char>(reply[pos])))
    {
        ++pos;
    }

    if (pos == start || (pos == start + 1 && (reply[start] == '+' || reply[start] == '-')))
    {
        return -1;
    }

    return std::atoi(reply.substr(start, pos - start).c_str());
}

std::string parseDashboardPayload(const std::string &reply)
{
    const auto open = reply.find('{');
    const auto close = reply.rfind('}');
    if (open != std::string::npos && close != std::string::npos && close >= open)
    {
        return reply.substr(open, close - open + 1);
    }
    return reply;
}

std::mutex g_dashboard_log_mutex;

void logDashboardRequest(const char *cmd)
{
    std::lock_guard<std::mutex> lock(g_dashboard_log_mutex);
    std::cout << "[dashboard] request: " << cmd << std::endl;
}

void logDashboardResult(const char *cmd, int32_t err_id, const std::string &reply)
{
    std::lock_guard<std::mutex> lock(g_dashboard_log_mutex);
    std::cout << "[dashboard] result: cmd=" << cmd << ", err_id=" << err_id << ", reply=" << reply << std::endl;
}

void logDashboardResultWithPayload(const char *cmd, int32_t err_id, const std::string &reply, const std::string &payload)
{
    std::lock_guard<std::mutex> lock(g_dashboard_log_mutex);
    std::cout << "[dashboard] result: cmd=" << cmd << ", err_id=" << err_id << ", reply=" << reply << ", payload=" << payload << std::endl;
}
}  // namespace
CRCommanderRos2::CRCommanderRos2(const std::string &ip)
    : current_joint_{}, tool_vector_{}, is_running_(false)
{
    is_running_ = false;
    real_time_data_ = std::make_shared<RealTimeData>();
    real_time_tcp_ = std::make_shared<TcpClient>(ip, 30004);
    dash_board_tcp_ = std::make_shared<TcpClient>(ip, 29999);
}

CRCommanderRos2::~CRCommanderRos2()
{
    is_running_ = false;
    thread_->join();
}

void CRCommanderRos2::getCurrentJointStatus(double *joint)
{
    mutex_.lock();
    memcpy(joint, current_joint_, sizeof(current_joint_));
    mutex_.unlock();
}

void CRCommanderRos2::getToolVectorActual(double *val)
{
    mutex_.lock();
    memcpy(val, tool_vector_, sizeof(tool_vector_));
    mutex_.unlock();
}

void CRCommanderRos2::recvTask()
{
    uint32_t has_read;
    while (is_running_)
    {
        if (real_time_tcp_->isConnect())
        {
            try
            {
                uint8_t *tmpData = reinterpret_cast<uint8_t *>(real_time_data_.get());
                if (real_time_tcp_->tcpRecv(tmpData, sizeof(RealTimeData), has_read, 5000))
                {

                    if (real_time_data_->len != 1440)
                        continue;

                    mutex_.lock();
                    for (uint32_t i = 0; i < 6; i++)
                        current_joint_[i] = deg2Rad(real_time_data_->q_actual[i]);

                    memcpy(tool_vector_, real_time_data_->tool_vector_actual, sizeof(tool_vector_));
                    mutex_.unlock();
                }
                else
                {
                    std::cout << "tcp recv timeout" << std::endl;
                }
            }
            catch (const TcpClientException &err)
            {
                real_time_tcp_->disConnect();
                std::cout << "tcp recv error :" << std::endl;
            }
        }
        else
        {
            try
            {
                real_time_tcp_->connect();
            }
            catch (const TcpClientException &err)
            {
                std::cout << "tcp recv Error : %s" << std::endl;
                sleep(3);
            }
        }

        if (!dash_board_tcp_->isConnect())
        {
            try
            {
                dash_board_tcp_->connect();
            }
            catch (const TcpClientException &err)
            {

                std::cout << "tcp recv ERROR : %s" << std::endl;
                sleep(3);
            }
        }
    }
}

void CRCommanderRos2::init()
{
    try
    {
        is_running_ = true;
        thread_ = std::unique_ptr<std::thread>(new std::thread(&CRCommanderRos2::recvTask, this));
    }
    catch (const TcpClientException &err)
    {
        std::cout << "Commander : %s" << std::endl;
    }
}
int stringToInt(const std::string& str) {
    return std::atoi(str.c_str());
}
void CRCommanderRos2::doTcpCmd(std::shared_ptr<TcpClient> &tcp, const char *cmd, int32_t &err_id,
                               std::vector<std::string> &result)
{
    std::ignore = result;
    try
    {
        err_id = -1;
        logDashboardRequest(cmd);
        tcp->tcpSend(cmd, strlen(cmd));
        const std::string reply = readDashboardReply(tcp);
        err_id = parseDashboardErrorId(reply);
        logDashboardResult(cmd, err_id, reply);
    }
    catch (const std::logic_error &err)
    {
        std::cout << "[dashboard] exception: cmd=" << cmd << ", error=" << err.what() << std::endl;
    }
}


void CRCommanderRos2::doTcpCmd_f(std::shared_ptr<TcpClient> &tcp, const char *cmd, int32_t &err_id,std::string &mode_id,
                               std::vector<std::string> &result)
{
    std::ignore = result;
    try
    {
        err_id = -1;
        mode_id.clear();
        logDashboardRequest(cmd);
        tcp->tcpSend(cmd, strlen(cmd));
        const std::string reply = readDashboardReply(tcp);
        err_id = parseDashboardErrorId(reply);
        mode_id = parseDashboardPayload(reply);
        logDashboardResultWithPayload(cmd, err_id, reply, mode_id);
    }
    catch (const std::logic_error &err)
    {
        std::cout << "[dashboard] exception: cmd=" << cmd << ", error=" << err.what() << std::endl;
    }
}

bool CRCommanderRos2::callRosService(const std::string cmd, int32_t &err_id)
{
    try
    {
        std::vector<std::string> result_;
        doTcpCmd(this->dash_board_tcp_, cmd.c_str(), err_id, result_);
        return true;
    }
    catch (const TcpClientException &err)
    {
        std::cout << "%s" << std::endl;
        err_id = -1;
        return false;
    }
}
bool CRCommanderRos2::callRosService_f(const std::string cmd, int32_t &err_id,std::string &mode_id)
{
    try
    {
        std::vector<std::string> result_;
        doTcpCmd_f(this->dash_board_tcp_, cmd.c_str(), err_id,mode_id, result_);
        return true;
    }
    catch (const TcpClientException &err)
    {
        std::cout << "%s" << std::endl;
        err_id = -1;
        return false;
    }
}
bool CRCommanderRos2::callRosService(const std::string cmd, int32_t &err_id, std::vector<std::string> &result_)
{
    try
    {
        doTcpCmd(this->dash_board_tcp_, cmd.c_str(), err_id, result_);
        return true;
    }
    catch (const TcpClientException &err)
    {
        std::cout << "%s" << std::endl;
        err_id = -1;
        return false;
    }
}

bool CRCommanderRos2::isEnable() const
{
    return real_time_data_->robot_mode == 5;
}

bool CRCommanderRos2::isConnected() const
{
    return dash_board_tcp_->isConnect() && real_time_tcp_->isConnect();
}

uint16_t CRCommanderRos2::getRobotMode() const
{
    return real_time_data_->robot_mode;
}

std::shared_ptr<RealTimeData> CRCommanderRos2::getRealData() const
{
    return real_time_data_;
}
