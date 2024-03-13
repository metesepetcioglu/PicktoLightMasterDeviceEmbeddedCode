Örneğin 0;1;2;3;4;5;6;7;8;9 şeklinde USB'den gelen string verisini parçalıp bir diziye int karakteri olarak atama yapan fonksiyon:

```
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  uint8_t len = (uint8_t) *Len;
  memset(buffer, '\0', 64);
  memcpy(buffer, Buf, len);
  char temp_val[4];
  uint8_t j = 0, k = 0;

      for (uint8_t i = 0; i < 42; i++) {
          if ((buffer[i] == ';') && (k < 10)) {
              temp_val[j] = '\0'; // Null karakterle sonlandır
              rx_data[k] = atoi(temp_val);
              k++;
              j = 0;
          }
          else if (j < 3) { // Diziyi taşmadan veri eklemek için boyut kontrolü yapın
              temp_val[j] = buffer[i];
              j++;
          }

      }
  memset(Buf, '\0', len);


  return (USBD_OK);
  /* USER CODE END 6 */
}
```
