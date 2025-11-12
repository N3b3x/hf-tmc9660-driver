var group__TMC9660__BootloaderProtocol =
[
    [ "tmc9660::BootloaderCommandSPI", "structtmc9660_1_1BootloaderCommandSPI.html", [
      [ "toBuffer", "structtmc9660_1_1BootloaderCommandSPI.html#a0d9e3440552ac6d4dfe853a6ae6da5cd", null ],
      [ "command", "structtmc9660_1_1BootloaderCommandSPI.html#a2a5cdf0d558dcdf1fa335c1686915fb8", null ],
      [ "value", "structtmc9660_1_1BootloaderCommandSPI.html#a8c69ce98f95278985516802cf6d250a5", null ]
    ] ],
    [ "tmc9660::BootloaderCommandUART", "structtmc9660_1_1BootloaderCommandUART.html", [
      [ "toBuffer", "structtmc9660_1_1BootloaderCommandUART.html#a085f9fc5aed572737da7966e7b61dc96", null ],
      [ "command", "structtmc9660_1_1BootloaderCommandUART.html#a46a2b5e5fe591105fe0307f1308d90c8", null ],
      [ "deviceAddr", "structtmc9660_1_1BootloaderCommandUART.html#aad5e33d8521dedd4090210e6431ddca5", null ],
      [ "value", "structtmc9660_1_1BootloaderCommandUART.html#ab0d44746ea82213a9f15441384b5fc79", null ]
    ] ],
    [ "tmc9660::BootloaderReplySPI", "structtmc9660_1_1BootloaderReplySPI.html", [
      [ "fromBuffer", "structtmc9660_1_1BootloaderReplySPI.html#a838a4008e6a8b7a718a30ed6026ebfa0", null ],
      [ "getStatus", "structtmc9660_1_1BootloaderReplySPI.html#a8f68d92b74cb83968e38f37b3a190d67", null ],
      [ "isOK", "structtmc9660_1_1BootloaderReplySPI.html#a5d45d31ab6dc388d128d22de23f225c4", null ],
      [ "status", "structtmc9660_1_1BootloaderReplySPI.html#aa32ccc6e7eed91928901681246945884", null ],
      [ "value", "structtmc9660_1_1BootloaderReplySPI.html#ac5e142ec51400a3ba756a5b53c0cbdb0", null ]
    ] ],
    [ "tmc9660::BootloaderReplyUART", "structtmc9660_1_1BootloaderReplyUART.html", [
      [ "fromBuffer", "structtmc9660_1_1BootloaderReplyUART.html#a41f97be3ddbdf4b8271fb167c404d46d", null ],
      [ "getStatus", "structtmc9660_1_1BootloaderReplyUART.html#abf240ab3150e16d1b462c8eddb11cd44", null ],
      [ "isOK", "structtmc9660_1_1BootloaderReplyUART.html#af6daf9b09ed59438b866b36b9abdf5b6", null ],
      [ "verifyCRC", "structtmc9660_1_1BootloaderReplyUART.html#aa79efb03d4cba868584c6b6e122e7dce", null ],
      [ "deviceAddr", "structtmc9660_1_1BootloaderReplyUART.html#ae684036bb213e6dd54e5d601f95c1e59", null ],
      [ "hostAddr", "structtmc9660_1_1BootloaderReplyUART.html#a121c4af4e6547770c8709cfca5edc9ac", null ],
      [ "status", "structtmc9660_1_1BootloaderReplyUART.html#afc20f4771de8168bc707103944cb2103", null ],
      [ "value", "structtmc9660_1_1BootloaderReplyUART.html#a86053b1a6c6949e86d0e6f5e5cb64955", null ]
    ] ],
    [ "tmc9660::BootloaderCommand", "group__TMC9660__BootloaderProtocol.html#ga941a785fa3d0a467511111f27ddfdcb0", [
      [ "tmc9660::BootloaderCommand::GET_INFO", "group__TMC9660__BootloaderProtocol.html#gga941a785fa3d0a467511111f27ddfdcb0ac95d2a6c5f29059e4ce8f19681800181", null ],
      [ "tmc9660::BootloaderCommand::GET_BANK", "group__TMC9660__BootloaderProtocol.html#gga941a785fa3d0a467511111f27ddfdcb0a6fff10bf2587cd114ff6311113b0a968", null ],
      [ "tmc9660::BootloaderCommand::SET_BANK", "group__TMC9660__BootloaderProtocol.html#gga941a785fa3d0a467511111f27ddfdcb0aef2bd38714d063b9727ecb5f5a2aa3cc", null ],
      [ "tmc9660::BootloaderCommand::GET_ADDRESS", "group__TMC9660__BootloaderProtocol.html#gga941a785fa3d0a467511111f27ddfdcb0a2c6a327ccec6b34078805f70e7ff46a7", null ],
      [ "tmc9660::BootloaderCommand::SET_ADDRESS", "group__TMC9660__BootloaderProtocol.html#gga941a785fa3d0a467511111f27ddfdcb0a4df54c1cb770ad7727befd34b054d61d", null ],
      [ "tmc9660::BootloaderCommand::READ_32", "group__TMC9660__BootloaderProtocol.html#gga941a785fa3d0a467511111f27ddfdcb0a63bd429758b357e222510ed896c29be5", null ],
      [ "tmc9660::BootloaderCommand::READ_32_INC", "group__TMC9660__BootloaderProtocol.html#gga941a785fa3d0a467511111f27ddfdcb0a07a56c09370a8504144f718569b87225", null ],
      [ "tmc9660::BootloaderCommand::READ_16", "group__TMC9660__BootloaderProtocol.html#gga941a785fa3d0a467511111f27ddfdcb0a2b25df9a61d99f8132be018bb3550e6e", null ],
      [ "tmc9660::BootloaderCommand::READ_16_INC", "group__TMC9660__BootloaderProtocol.html#gga941a785fa3d0a467511111f27ddfdcb0a2e4c5b8d85864b9c4eebb8493fb12951", null ],
      [ "tmc9660::BootloaderCommand::READ_8", "group__TMC9660__BootloaderProtocol.html#gga941a785fa3d0a467511111f27ddfdcb0af88db2df25242062cfc4f25281048972", null ],
      [ "tmc9660::BootloaderCommand::READ_8_INC", "group__TMC9660__BootloaderProtocol.html#gga941a785fa3d0a467511111f27ddfdcb0afe461e4aa5bdbf4375e119eac2bbffc6", null ],
      [ "tmc9660::BootloaderCommand::WRITE_32", "group__TMC9660__BootloaderProtocol.html#gga941a785fa3d0a467511111f27ddfdcb0a87a2b6aa46f65de3404d21e2ba3ff677", null ],
      [ "tmc9660::BootloaderCommand::WRITE_32_INC", "group__TMC9660__BootloaderProtocol.html#gga941a785fa3d0a467511111f27ddfdcb0aba0729b1bc5a3b8391493531fbbcbdae", null ],
      [ "tmc9660::BootloaderCommand::WRITE_16", "group__TMC9660__BootloaderProtocol.html#gga941a785fa3d0a467511111f27ddfdcb0ab7c1c772e5dce85f52515d163243ed7a", null ],
      [ "tmc9660::BootloaderCommand::WRITE_16_INC", "group__TMC9660__BootloaderProtocol.html#gga941a785fa3d0a467511111f27ddfdcb0a4597cbb1b4fb87ded75cc35841b5e38f", null ],
      [ "tmc9660::BootloaderCommand::WRITE_8", "group__TMC9660__BootloaderProtocol.html#gga941a785fa3d0a467511111f27ddfdcb0a23963a90163c1febc120d6c0d610416e", null ],
      [ "tmc9660::BootloaderCommand::WRITE_8_INC", "group__TMC9660__BootloaderProtocol.html#gga941a785fa3d0a467511111f27ddfdcb0af041787aca1789b6fa382ea995040a95", null ],
      [ "tmc9660::BootloaderCommand::NO_OP", "group__TMC9660__BootloaderProtocol.html#gga941a785fa3d0a467511111f27ddfdcb0a14b52fa379dea9171c3eb958d7fa5795", null ],
      [ "tmc9660::BootloaderCommand::OTP_LOAD", "group__TMC9660__BootloaderProtocol.html#gga941a785fa3d0a467511111f27ddfdcb0ab853ce737ad114247dd1dc4591d493f4", null ],
      [ "tmc9660::BootloaderCommand::OTP_BURN", "group__TMC9660__BootloaderProtocol.html#gga941a785fa3d0a467511111f27ddfdcb0a242047f0b587586b4113667f3acbac9b", null ],
      [ "tmc9660::BootloaderCommand::MEM_IS_CONFIGURED", "group__TMC9660__BootloaderProtocol.html#gga941a785fa3d0a467511111f27ddfdcb0a6c5f29036941787c7a47a01c2f291466", null ],
      [ "tmc9660::BootloaderCommand::MEM_IS_CONNECTED", "group__TMC9660__BootloaderProtocol.html#gga941a785fa3d0a467511111f27ddfdcb0a519af632f125e5ab942ccea303802b30", null ],
      [ "tmc9660::BootloaderCommand::FLASH_SEND_CMD", "group__TMC9660__BootloaderProtocol.html#gga941a785fa3d0a467511111f27ddfdcb0a52c0478386ff2c235bf03c5882b95159", null ],
      [ "tmc9660::BootloaderCommand::FLASH_ERASE_SECTOR", "group__TMC9660__BootloaderProtocol.html#gga941a785fa3d0a467511111f27ddfdcb0aa86c3905134ee9fe40942d2da7664a49", null ],
      [ "tmc9660::BootloaderCommand::MEM_IS_BUSY", "group__TMC9660__BootloaderProtocol.html#gga941a785fa3d0a467511111f27ddfdcb0a3c123c2ce736b95d229aac32cab81862", null ],
      [ "tmc9660::BootloaderCommand::BOOTSTRAP_RS485", "group__TMC9660__BootloaderProtocol.html#gga941a785fa3d0a467511111f27ddfdcb0a1202247ee7281217be83a51566d50419", null ]
    ] ],
    [ "tmc9660::BootloaderStatus", "group__TMC9660__BootloaderProtocol.html#ga9cbaf21724b4f9793c31bb96da5a98d7", [
      [ "tmc9660::BootloaderStatus::OK", "group__TMC9660__BootloaderProtocol.html#gga9cbaf21724b4f9793c31bb96da5a98d7ae0aa021e21dddbd6d8cecec71e9cf564", null ],
      [ "tmc9660::BootloaderStatus::CMD_NOT_FOUND", "group__TMC9660__BootloaderProtocol.html#gga9cbaf21724b4f9793c31bb96da5a98d7a59697eb816f6e6200846ce767d79d241", null ],
      [ "tmc9660::BootloaderStatus::INVALID_ADDR", "group__TMC9660__BootloaderProtocol.html#gga9cbaf21724b4f9793c31bb96da5a98d7a4bdcab0129348e511a0f7dea0cbcda5f", null ],
      [ "tmc9660::BootloaderStatus::INVALID_VALUE", "group__TMC9660__BootloaderProtocol.html#gga9cbaf21724b4f9793c31bb96da5a98d7ad8f24f388e990b9ccf8905b7993b99ae", null ],
      [ "tmc9660::BootloaderStatus::INVALID_BANK", "group__TMC9660__BootloaderProtocol.html#gga9cbaf21724b4f9793c31bb96da5a98d7a4166ef107996f39e4e246960cb2a9d5f", null ],
      [ "tmc9660::BootloaderStatus::BUSY", "group__TMC9660__BootloaderProtocol.html#gga9cbaf21724b4f9793c31bb96da5a98d7a802706a9238e2928077f97736854bad4", null ],
      [ "tmc9660::BootloaderStatus::MEM_UNCONFIGURED", "group__TMC9660__BootloaderProtocol.html#gga9cbaf21724b4f9793c31bb96da5a98d7a38a1e989e5582b624ee8c82b353eea30", null ],
      [ "tmc9660::BootloaderStatus::OTP_ERROR", "group__TMC9660__BootloaderProtocol.html#gga9cbaf21724b4f9793c31bb96da5a98d7adccd34766cbae8b3354e497e112a27ba", null ],
      [ "tmc9660::BootloaderStatus::SESSION_START", "group__TMC9660__BootloaderProtocol.html#gga9cbaf21724b4f9793c31bb96da5a98d7a20159b7454cb5cdaaefeb02195a99043", null ],
      [ "tmc9660::BootloaderStatus::CMD_NOT_AVAILABLE", "group__TMC9660__BootloaderProtocol.html#gga9cbaf21724b4f9793c31bb96da5a98d7ad9cf3a4524a6431312c5dfce452fefbf", null ],
      [ "tmc9660::BootloaderStatus::BOOTLOADER_RESUMED", "group__TMC9660__BootloaderProtocol.html#gga9cbaf21724b4f9793c31bb96da5a98d7a12181e1139ca26ef787811b6bb4d1ecd", null ]
    ] ],
    [ "tmc9660::InfoQuery", "group__TMC9660__BootloaderProtocol.html#gaf4e8a4c74acf7f0c687427b7fc2698f2", [
      [ "tmc9660::InfoQuery::CHIP_TYPE", "group__TMC9660__BootloaderProtocol.html#ggaf4e8a4c74acf7f0c687427b7fc2698f2a118600a37b2efdb0b180425ee70f5b20", null ],
      [ "tmc9660::InfoQuery::BL_VERSION", "group__TMC9660__BootloaderProtocol.html#ggaf4e8a4c74acf7f0c687427b7fc2698f2ab0b4029d33e32fe34019e6faec4a5daa", null ],
      [ "tmc9660::InfoQuery::FEATURES", "group__TMC9660__BootloaderProtocol.html#ggaf4e8a4c74acf7f0c687427b7fc2698f2ab1fa6b4767532c2e890022101221bd08", null ],
      [ "tmc9660::InfoQuery::GIT_INFO", "group__TMC9660__BootloaderProtocol.html#ggaf4e8a4c74acf7f0c687427b7fc2698f2a8b79c13c76697ca3e733bb68e0cc67d5", null ],
      [ "tmc9660::InfoQuery::CHIP_VERSION", "group__TMC9660__BootloaderProtocol.html#ggaf4e8a4c74acf7f0c687427b7fc2698f2afc54cc5168d94ca409e12befda8c2a4a", null ],
      [ "tmc9660::InfoQuery::CHIP_FREQUENCY", "group__TMC9660__BootloaderProtocol.html#ggaf4e8a4c74acf7f0c687427b7fc2698f2af9aff27ab1927063979a5e6b03c68a64", null ],
      [ "tmc9660::InfoQuery::CONFIG_MEM_START", "group__TMC9660__BootloaderProtocol.html#ggaf4e8a4c74acf7f0c687427b7fc2698f2a682953ed3025ca930f537ff520acea38", null ],
      [ "tmc9660::InfoQuery::CONFIG_MEM_SIZE", "group__TMC9660__BootloaderProtocol.html#ggaf4e8a4c74acf7f0c687427b7fc2698f2a1c10d8b2f69deab6114fc3aa188c581f", null ],
      [ "tmc9660::InfoQuery::OTP_MEM_SIZE", "group__TMC9660__BootloaderProtocol.html#ggaf4e8a4c74acf7f0c687427b7fc2698f2a80aec1a2097b1b78f549f3cbc8ae0e52", null ],
      [ "tmc9660::InfoQuery::I2C_MEM_SIZE", "group__TMC9660__BootloaderProtocol.html#ggaf4e8a4c74acf7f0c687427b7fc2698f2ace6c3bc084e62afe3f94d8cb42b9074d", null ],
      [ "tmc9660::InfoQuery::SPI_MEM_SIZE", "group__TMC9660__BootloaderProtocol.html#ggaf4e8a4c74acf7f0c687427b7fc2698f2a7ff94d3b9aca70d727bfbc9956fcc4c0", null ],
      [ "tmc9660::InfoQuery::PARTITION_VERSION", "group__TMC9660__BootloaderProtocol.html#ggaf4e8a4c74acf7f0c687427b7fc2698f2a8885ddf4239bc6094f9709911f0d5ac0", null ],
      [ "tmc9660::InfoQuery::SPI_MEM_PARTITIONS", "group__TMC9660__BootloaderProtocol.html#ggaf4e8a4c74acf7f0c687427b7fc2698f2a9069906fe5830eeb949e95fd3ed8a0ec", null ],
      [ "tmc9660::InfoQuery::I2C_MEM_PARTITIONS", "group__TMC9660__BootloaderProtocol.html#ggaf4e8a4c74acf7f0c687427b7fc2698f2adfdf508fe6f42769e4c4a9dc1d2e6471", null ],
      [ "tmc9660::InfoQuery::CHIP_VARIANT", "group__TMC9660__BootloaderProtocol.html#ggaf4e8a4c74acf7f0c687427b7fc2698f2ab3ad534c805b989c6f4e949e73086778", null ]
    ] ],
    [ "tmc9660::MemoryBank", "group__TMC9660__BootloaderProtocol.html#ga59bad2e9f988290ca3c01c8b3b535cae", [
      [ "tmc9660::MemoryBank::RAM", "group__TMC9660__BootloaderProtocol.html#gga59bad2e9f988290ca3c01c8b3b535caeae53619c1fe611a51eeeb8d148ba6e532", null ],
      [ "tmc9660::MemoryBank::OTP", "group__TMC9660__BootloaderProtocol.html#gga59bad2e9f988290ca3c01c8b3b535caead9d1e60e50119d9752001d4196ee6b3c", null ],
      [ "tmc9660::MemoryBank::SPI_FLASH", "group__TMC9660__BootloaderProtocol.html#gga59bad2e9f988290ca3c01c8b3b535caea424eafe560b4e1cf1739bd96f6a5bda9", null ],
      [ "tmc9660::MemoryBank::I2C_EEPROM", "group__TMC9660__BootloaderProtocol.html#gga59bad2e9f988290ca3c01c8b3b535caeafe03a69b97c89a822b1f08f5eb087eb7", null ],
      [ "tmc9660::MemoryBank::RESERVED", "group__TMC9660__BootloaderProtocol.html#gga59bad2e9f988290ca3c01c8b3b535caea83c7f2aa8c3ac10ed8beb75cad162827", null ],
      [ "tmc9660::MemoryBank::CONFIG", "group__TMC9660__BootloaderProtocol.html#gga59bad2e9f988290ca3c01c8b3b535caea73e99d350a4aa6f1a5af04ec29173f73", null ]
    ] ]
];