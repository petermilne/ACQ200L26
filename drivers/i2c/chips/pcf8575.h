/*
 * pcf8575.h - Kernel API to pcf8575
 */

u16 pcf8575_read(struct device *dev);
u16 pcf8575_write(struct device *dev, u16 data);
