sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up
sudo ip link set can1 up

# cansend can0 001#FFFFFFFFFFFFFFFC
# cansend can1 001#FFFFFFFFFFFFFFFC

# 卸载当前占用的cdc_acm驱动（如果需要）
# echo "1-3:1.3" | sudo tee /sys/bus/usb/drivers/cdc_acm/unbind

# 尝试绑定gs_usb到不同接口
# for i in {0..4}; do
#     echo "1-3:1.$i" | sudo tee /sys/bus/usb/drivers/gs_usb/bind 2>/dev/null
#     sleep 1
#     if ip link show can0 2>/dev/null; then
#         echo "成功！CAN接口已绑定到 1-3:1.$i"
#         exit 0
#     fi
#     echo "1-3:1.$i 绑定失败" | sudo tee /sys/bus/usb/drivers/gs_usb/unbind 2>/dev/null
# done
# echo "所有接口绑定失败，请尝试其他方法"
