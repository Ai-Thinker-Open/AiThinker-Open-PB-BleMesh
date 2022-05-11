# SKD Release Notes

## ChipSet

***PHY6222/PHY6252***

---

## SDK Version

***PHY62XX_SDK_3.X.X***

---

### **Version**: PHY62XX_SDK_3.1.1

### **Change List**

### **[components]**

    driver:
        ota_flash   : update ota_flash.c/slb.c to support fast slb boot
        flash       : add flash api to get flash id
        pwrmgr      : 1.add _wdt_init in wakuep_init1
                      2.add osal_idle_task in hal_pwrmgr_init when wdt enabled
                      3.watchdog_config for wtd rest cyc config and enable wdt
                      4.feed wdt in osal_idle_task
    mesh:           : support to add proxy filter list api

### **[example]**
    ble_mesh        : bugfix blebrr queue full (bleMemAllocError status process)
    ble_peripheral  : simpleBleperipheral procedure optimize
                      update system clock to DBL 32M

### **[lib]**

    rf.lib          : 1.optimize sleep wakeup procedure,add wakeupinit tracking for 32M DBL 
                      2.fix large slaveLatency issue
                      3.fix 2M PHY extpreamble issue
                      4.fix secondscan & secondinit CSA2 issue
    mesh_lib        : 1,update ltrn replay cache process when replay cache is full
					  2,bugfix SAR Context Allocation fail

### **[misc]**

---

### **Version**: PHY62XX_SDK_3.1.0

### **Change List**

### **[components]**

    driver:
        ppsp        : bugfix ppsp data format bug
        ota_flash   : 1,fix ota hardfault bug(100% progress) 
                      2,update slb service 
        flash       : add spif lock in flash.c
        pwrmgr      : CFG_SRAM_RETENTION_LOW_CURRENT_LDO_ENABLE,default
        i2c         : 1.disable i2c clock when deinit
                      2.enable i2c clock when i2c address update
    mesh:           : 1.set em timer handle to default value
                      2.bugfix get subscription list error
                      3.bugfix stop timer failed when event id can't find
                      4.add debug information for queue full
                      5.rename phyplus model to vendor model

### **[example]**

    ble_mesh        : 1.bugfix mesh switch proxy beacon failed
                      2.add 2M mode

### **[lib]**

    rf.lib          : 1.add load MAC Address from chip Madd function
                      2.update wakeupinit tracking,optimize sleep wakeup procedure
                      3.add ll adv control patch to master
                      4.update rf driver
    mesh_lib        : 1.improve provision ratio
                      2.separate gatt and adv bearear re-transmit time
                      3.set ms_provisioner_addr to unassigned at reset case

### **[misc]**

    bb_rom_sym_m0   : 1.delete more common rom symbol
                      2.add llConnTerminate0 symbol

---
### **Version**: PHY62XX_SDK_3.0.9

### **Change List**

### **[components]**

    driver:
        adc         : optimize adc attenuation mode correction algorithm 
        pwrmgr      : add rf timer irq prio restore
        spi         : optimize spi slvae config
        clock       : optimize WaitMs(use WaitRTCCount)
        bsp_button  : add bsp_button code,it is a key process middleware which use gpio or kscan
        ota_flash   : add crc check when in single no fct mode
        flash       : add load MAC from chipmaddr

    profiles:
        gapbondmgr  : set gapBondMgr_TaskID default value to INVALID_TASK_ID

### **[example]**

    ble_multi       : 1.update multi-role scheduler
                      2.max support 5 link（2 slave and 3 master）
                      3.support DLE , MTU , SMP
    
    ble_mesh        : 1.add ali genie mesh project(include 3KeySwitch,curtain...)
                      2.change scatter_load for ota
                      3.improve provision ratio
                      4.fix proxy timing bug(GAP_DeviceDiscoveryRequest Retry)
                      5.fix mesh light provision failed bug when not use easybonding
                      6.delete sm task
    
    ble_peripheral  : 
        bleUart_AT  : 1.move cliface into source dir, comment out pwrmgr.o file in scatter file 
                      2.simpleBleperipheral:fix scanrsp data bit 
        OTA         : add ota security boot and slb ota

### **[lib]**

    rf.lib          : 1.update rf driver
                      2.add fastadv
                      3.optimize sleep wakeup procedure
                      4.update wakeupinit tracking
    mesh_lib        : 1.fix mesh reset cannot restart bug(provsioner and device)
                      2.unuse friend/lpn function
                      3.unuse MS_ACCESS_PUBLISH_TIMER_SUPPORT function
                      4.change mesh config address(max 4K bytes)
                      5.change nvs init funtion(can choose flash address)
    ble_host        : 1.MTU SIZE marcro define default 247
                      2.ATT/GATT API change uint8 to uint16
    ble_host_multi5 : for ble_multi example

### **[misc]**

    bb_rom_sym_m0   : 1.delete more common rom symbol
                      2.add g_llAdvMode symbol

---

### **Version**: PHY62XX_SDK_3.0.8

### **Change List**

### **[components]**

    driver:
        adc         : update adc_Lambda for adc value calc
        flash       : 1.add cache reset in HAL_CACHE_ENTER/EXIT_BYPASS_SECTION
                      2.add cache bypass in hal_flash_erase_sector/block64/all
        i2c         : fix scl/sda fmux error in i2c_common.c
        kscan       : support low power sleep wakeup
        log         : add LOG_DUMP_BYTE
        pwrmgr      : 1.support multi-io wakeup in standy mode
                      2.turn on CLK_COM , modifed the DEF_CLKG_CONFIG_1
        spi         : fix dma usage issue
        spiflash    : add spiflash bus busy check in spifflash_sector_erase/32KB/64KB
        uart        : fix dma usage issue
    
    ethermind:
        add mesh components
    
    osal:
        osal_snv    : config USE_FS=1 by default, clear NO_FS osal_svn api

    profiles:
        gapbondmgr  : support unlimit GAP_BONDING number by config
                      GAP_BOND_MGR_INDEX_REPLACE

### **[example]**

    ble_central     : add simpleBleCentral demo
    
    ble_mesh        : 1.add aliGenie_mesh_light
                      2.add aliGenie_mesh_multi
                      3.add mesh_gateway
                      4.add mesh_light
                      5.add mesh_switch 
    
    ble_peripheral  : add bleUart_AT 

### **[lib]**

    rf.lib          : 1.update rf driver 
                      2.optimize sleep wakeup time
                      3.add TRNG api(rf_phy_driver.h)
    rf_mst.lib      : for simpleBleCentral demo and ble mesh example

### **[misc]**

    bb_rom_sym_m0   : add more common rom symbol
    jump_table      : add _hard_fault handle for debug

---

### **Version**: PHY62XX_SDK_3.0.7

### **Change List**

    1. pwrmgr:   add hal_pwrmgr_enter_standby api 
    2. dtm: support 125K/500K, ext dtm api
    3. flash: add fix hal_cache_tag_flush bug and add spif_config in hal_cache_init->hw_spif_cache_init
    4. flash: add SPIF polling interval config in hw_spif_cache_config
    5. hiddev: updated for mobile phone compatibility
