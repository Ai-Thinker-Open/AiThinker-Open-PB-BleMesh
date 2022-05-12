# 前言

下表总结了安信可模组在本仓库各版本中的支持状态，每个模组对应的分支有所不同:
|    分支/模组     |       PB-01         |    PB-02     |        PB-03系列     |                                               
|:----------- |:---------------------: | :---------------------:| :---------------------:|
|  master           | 支持 | 支持 |      不支持 |      
|  release/pb-03          | 不支持 |不支持 |   支持 |       

## SDK Release Notes 更新记录

### **Version**:  2.1.2

### **Change List**:
    1. update sdk_build.yml for multi_stack.lib copy. 
    2. delete redundant LOG and gpio_toggle in ll controller code.
    3. restore rx2tx,tx2rx,scan_req_to_scan_rsp timing.
    4. Bugfix smp test，smp push and multi role dtm.
    5. bugfix scan Advertising Data length always plus 1. 
    6. bugfix application task do not deallocate system osal message.
    7. disable spi cs pin function(deafult is io input) for spi slave transmit.
    8. change uart fullmux order to fix uart receive error bug.
    9. add multi role hid profile.
    10. remove GAPBondMgr_Register and set gapBondMgr_TaskID default value to INVALID_TASK_ID.
    11. add bleuart_at example.
    12. modify sbc use rf_mst.lib.
    13. adjust t-ifs for optimize scan_req,scan_rsp and connect_req.
    14. add aliGenie_bleMesh project sample(total 8 project)
    15. chenge provision pdu rtx count and timer,for improve provision success ratio 
    16. bug fix net_trigger_tx failed when NET Tx Q FULL
    17. bug fix Seq_Zero 13 bit overrun
    18. add MS_access_ps_store_all_record function to restore mesh config information
    19. add start unicast address for alloc
    20. add ota service(mesh project)
    21. fix some bugs


---
### **Version**:   2.1.1

### **Change List**:
    1. spi:    optimize spi code struct,impove its compatibility. 
    2. uart：  fix bug of bleUart-RawPass which cannot sleep.
    3. clock:  add several api,use more easily.
    4. OTA internal flash project modified scatter_loader，fix boot bug when image size larger than 100KB
    5. add example ble_multi/simpleBleMultiConnection 
    6. mesh_light support pbadv/pbgatt
    7. add key refresh feature
    8. add IV update feature
    9. optimize data transfer efficiency，256bytes/700-800ms
    10. add low power feature
    11. fix miss segment data bug(seqzero overrun) 
    12. add phymodel（phyplus vendor model）
    13. gateway support 256 nodes max
    14. node support multi element(primary/secondry)，5 max
    15. gateway/node support heartbeat feature
    16. add ota service
    17. optimize gateway provision ratio and time
    18. fix some bugs

---
### **Version**:   2.1.0

### **Change List**:
    1. add mesh switch example for mesh ligth remote
    2. opitime mesh config parameter storge 
    3. Set network retransmitter counter as 2 when dest addr is group addr and packet segment is necessary
    4. modified hsl set delay timing(1s->350ms)
    5. move mesh fs MARCO Define to access_internal.h
    6. fix bug: No response ack when target address is group address
    7. fix bug: ltrn report error ttl vault to higher layer
