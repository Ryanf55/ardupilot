#include "AP_DDS_Client.h"
#include <GCS_MAVLink/GCS.h>

#if AP_DDS_UDP_ENABLED

#include <errno.h>

/*
  open connection on UDP
 */
bool AP_DDS_Client::udp_transport_open(uxrCustomTransport *t)
{
    AP_DDS_Client *dds = (AP_DDS_Client *)t->args;

    if (dds->udp.socket != nullptr) {
        return true;
    }
    auto *sock = new SocketAPM(true);
    if (sock == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s:%u", __FILE__, __LINE__);
        return false;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Connecting to %s:%u", dds->udp.ip.get_str(), (unsigned)dds->udp.port.get());

    if (!sock->connect(dds->udp.ip.get_str(), dds->udp.port.get())) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s:%u", __FILE__, __LINE__);
        delete sock;
        return true;
    }
    dds->udp.socket = sock;
    return true;
}

/*
  close UDP connection
 */
bool AP_DDS_Client::udp_transport_close(uxrCustomTransport *t)
{
    AP_DDS_Client *dds = (AP_DDS_Client *)t->args;
    delete dds->udp.socket;
    dds->udp.socket = nullptr;
    return true;
}

/*
  write on UDP
 */
size_t AP_DDS_Client::udp_transport_write(uxrCustomTransport *t, const uint8_t* buf, size_t len, uint8_t* error)
{

    AP_DDS_Client *dds = (AP_DDS_Client *)t->args;
    dds->udp_transport_open(t);
    if (dds->udp.socket == nullptr) {
        *error = EINVAL;
        return 0;
    }
    const ssize_t ret = dds->udp.socket->send(buf, len);
    if (ret <= 0) {
        *error = errno;
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Failed to write %u ret=%d errno =%u:", unsigned(len), int(ret), unsigned(*error));
        return 0;
    }
    return ret;
}

/*
  read from UDP
 */
size_t AP_DDS_Client::udp_transport_read(uxrCustomTransport *t, uint8_t* buf, size_t len, int timeout_ms, uint8_t* error)
{
    AP_DDS_Client *dds = (AP_DDS_Client *)t->args;
    dds->udp_transport_open(t);
    if (dds->udp.socket == nullptr) {
        *error = EINVAL;
        return 0;
    }
    const ssize_t ret = dds->udp.socket->recv(buf, len, timeout_ms + 100);
    static ssize_t last_recv_ret = 0;
    if (ret <= 0) {
        *error = errno;

        if(ret != last_recv_ret) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Failed to read %u ret=%d errno =%u, timeout=%d", unsigned(len), int(ret), unsigned(*error), timeout_ms);
            last_recv_ret = ret;
        }
        return 0;
    }
    return ret;
}

/*
  initialise UDP connection
 */
bool AP_DDS_Client::ddsUdpInit()
{
    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "DDS UDP INIT %s:%u", __FILE__, __LINE__);
    // setup a non-framed transport for UDP
    uxr_set_custom_transport_callbacks(&udp.transport, false,
                                       udp_transport_open,
                                       udp_transport_close,
                                       udp_transport_write,
                                       udp_transport_read);

    if (!uxr_init_custom_transport(&udp.transport, (void*)this)) {
        return false;
    }
    comm = &udp.transport.comm;
    return true;
}
#endif // AP_DDS_UDP_ENABLED
