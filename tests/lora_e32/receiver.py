#!/usr/bin/env python3
"""LoRa E32 receiver on a CP2102 USB-TTL adapter: prints each packet on the PC.

Wiring (E32 -> CP2102, 5 pins): VCC->5V, GND->GND, TXD->RXD, RXD->TXD,
M0 and M1->GND (normal mode), AUX and 3V3 unconnected.

Run from the repository root:
    python3 tests/lora_e32/receiver.py
    python3 tests/lora_e32/receiver.py --port /dev/cu.usbserial-0001 --log lora.csv
    python3 tests/lora_e32/receiver.py --list
    python3 tests/lora_e32/receiver.py --info     # with M0 and M1 on the CP2102 3V3 pin
"""
import argparse
import csv
import os
import re
import sys
import time
from datetime import datetime

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit('Falta o pyserial: python3 -m pip install pyserial')

CP210X = (0x10C4, 0xEA60)
FACTORY = bytes.fromhex('C000001A1744')  # 433 MHz, UART 9600 8N1, 2.4 kbps on air
SEQUENCE = re.compile(r'#(\d+)')         # transmitter packets look like "LABSIS #12 t=34s"
UART_BAUDS = (1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200)
AIR_RATES = ('0.3', '1.2', '2.4', '4.8', '9.6', '19.2', '19.2', '19.2')
PARITY = ('8N1', '8O1', '8E1', '8N1')
POWER_T20D = (20, 17, 14, 10)
SILENCE_HINT_S = 10


def hexdump(data):
    return ' '.join('{:02X}'.format(b) for b in data) or '(nada)'


def printable(raw):
    return ''.join(chr(b) if 32 <= b < 127 else '\\x{:02X}'.format(b) for b in raw)


def list_all():
    ports = list(list_ports.comports())
    if not ports:
        print('Nenhuma porta serial encontrada.')
    for port in ports:
        tag = '   <- CP210x' if (port.vid, port.pid) == CP210X else ''
        print('{}  {}{}'.format(port.device, port.description, tag))


def choose_port(port):
    if port:
        return port
    found = [p for p in list_ports.comports() if (p.vid, p.pid) == CP210X]
    if len(found) == 1:
        return found[0].device
    if not found:
        sys.exit('Nenhum CP210x encontrado. Confira o USB ou passe --port (veja --list).')
    names = '\n'.join('  {}  {}'.format(p.device, p.description) for p in found)
    sys.exit('Há {} CP210x ligados (a placa do ESP32 também tem um). '
             'Escolha o do receptor com --port.\n{}'.format(len(found), names))


def open_port(port, baud, timeout):
    try:
        return serial.Serial(port, baud, timeout=timeout)
    except serial.SerialException as error:
        sys.exit('Não consegui abrir {} ({}). Confira o nome com --list e feche monitores '
                 'seriais que estejam usando a porta.'.format(port, error))


def describe(params):
    """Summary of the 6 parameter bytes: HEAD ADDH ADDL SPED CHAN OPTION."""
    _, addh, addl, sped, chan, option = params
    baud = UART_BAUDS[(sped >> 3) & 7]
    return baud, [
        'endereço 0x{:02X}{:02X} | UART {} {} | ar {} kbps | canal 0x{:02X} = {} MHz'.format(
            addh, addl, baud, PARITY[sped >> 6], AIR_RATES[sped & 7], chan, 410 + (chan & 0x1F)),
        '{} | {} | FEC {} | potência: bits {:02b} ({} dBm no T20D)'.format(
            'transmissão fixa' if option & 0x80 else 'transparente',
            'push-pull' if option & 0x40 else 'dreno aberto',
            'ligado' if option & 0x04 else 'desligado', option & 3, POWER_T20D[option & 3]),
    ]


def exchange(ser, command, size):
    ser.reset_input_buffer()
    ser.write(command)
    ser.flush()
    answer = ser.read(size)
    time.sleep(0.1)  # AUX is not wired here: give the module time between commands
    return answer


def read_module(port, write_factory):
    """Configuration mode (M0 = M1 = 3V3) always talks 9600 8N1 and never transmits."""
    print('Lendo o E32 em {} (configuração, 9600 8N1).'.format(port))
    with open_port(port, 9600, 0.6) as ser:
        time.sleep(0.2)
        if write_factory:
            echoed = exchange(ser, FACTORY, 6)
            print('Gravando o padrão de fábrica C0 00 00 1A 17 44. Resposta {}'.format(hexdump(echoed)))
        version = exchange(ser, b'\xC3\xC3\xC3', 4)
        params = exchange(ser, b'\xC1\xC1\xC1', 6)
    print('Versão (C3 C3 C3):     {}'.format(hexdump(version)))
    print('Parâmetros (C1 C1 C1): {}'.format(hexdump(params)))
    if version == b'\xC3\xC3\xC3' and params == b'\xC1\xC1\xC1':
        print('Voltou o próprio comando: o TXD do CP2102 está ligado no RXD dele (loopback). '
              'O CP2102 e a porta funcionam, então o problema está no lado do E32.')
        return 0
    if len(params) != 6 or params[0] not in (0xC0, 0xC2):
        print('Sem resposta válida. Confira M0 e M1 no 3V3, TXD do E32 no RXD do CP2102, '
              'RXD do E32 no TXD do CP2102 e VCC no 5V. Para testar só o CP2102, tire o E32, '
              'ligue o TXD do CP2102 no RXD dele e rode --info de novo.')
        return 1
    baud, lines = describe(params)
    for line in lines:
        print('  ' + line)
    if params[1:] != FACTORY[1:]:
        print('  Diferente do padrão de fábrica (C0 00 00 1A 17 44). --padrao restaura.')
    extra = '' if baud == 9600 else ' e rode com --baud {}'.format(baud)
    print('OK. Volte M0 e M1 para o GND antes de receber{}.'.format(extra))
    return 0


class Counter:
    """Counts packets and the gaps in their "#n" numbering."""

    def __init__(self):
        self.received = self.numbered = self.lost = 0
        self.last = None
        self.start = time.monotonic()

    def update(self, text):
        self.received += 1
        match = SEQUENCE.search(text)
        if not match:
            return 0, ''
        number = int(match.group(1))
        self.numbered += 1
        gap, note = 0, ''
        if self.last is not None and number > self.last + 1:
            gap = number - self.last - 1
            self.lost += gap
            note = '   ({} perdido{})'.format(gap, 's' if gap > 1 else '')
        elif self.last is not None and number <= self.last:
            note = '   (transmissor reiniciou a contagem)'
        self.last = number
        return gap, note

    def summary(self):
        elapsed = int(time.monotonic() - self.start)
        expected = self.numbered + self.lost
        rate = 100.0 * self.lost / expected if expected else 0.0
        return 'Recebidos {}, perdidos {} ({:.0f}%) em {}:{:02d}'.format(
            self.received, self.lost, rate, elapsed // 60, elapsed % 60)


def receive(port, baud, log_path):
    counter = Counter()
    log = writer = None
    if log_path:
        new_file = not os.path.exists(log_path) or os.path.getsize(log_path) == 0
        log = open(log_path, 'a', newline='', encoding='utf-8')
        writer = csv.writer(log)
        if new_file:
            writer.writerow(['hora', 'pacote', 'perdidos_antes'])

    def show(raw):
        raw = raw.rstrip(b'\r')
        if not raw:
            return
        text = printable(raw)
        now = datetime.now()
        gap, note = counter.update(text)
        print('{}  {}{}'.format(now.strftime('%H:%M:%S.%f')[:-3], text, note), flush=True)
        if writer:
            writer.writerow([now.isoformat(timespec='milliseconds'), text, gap])
            log.flush()

    ser = open_port(port, baud, 0.05)
    print('Ouvindo {} a {} 8N1. Ctrl+C encerra.'.format(port, baud), flush=True)
    pending = bytearray()
    last_byte = time.monotonic()
    hinted = False
    try:
        while True:
            chunk = ser.read(256)
            now = time.monotonic()
            if chunk:
                pending += chunk
                last_byte = now
                while b'\n' in pending:
                    line, _, rest = bytes(pending).partition(b'\n')
                    pending = bytearray(rest)
                    show(line)
            elif pending and now - last_byte > 0.15:  # packet without a newline
                show(bytes(pending))
                pending.clear()
            if not hinted and not counter.received and now - counter.start > SILENCE_HINT_S:
                hinted = True
                print('Nada em {} s. Confira se a ESP32 está ligada e transmitindo, as duas antenas, '
                      'M0 e M1 no GND e o mesmo canal nos dois módulos (--info).'.format(SILENCE_HINT_S),
                      flush=True)
    except KeyboardInterrupt:
        pass
    except serial.SerialException as error:
        print('A porta caiu ({}). O CP2102 foi desconectado?'.format(error))
    finally:
        ser.close()
        if log:
            log.close()
        print('\n' + counter.summary())
    return 0


def main():
    parser = argparse.ArgumentParser(description='Recebe pacotes do E32 (433 MHz) por um conversor CP2102.')
    parser.add_argument('--port', help='porta do CP2102 (padrão: o único CP210x ligado)')
    parser.add_argument('--baud', type=int, default=9600, help='UART do E32 receptor (padrão 9600)')
    parser.add_argument('--log', metavar='ARQUIVO.csv', help='também salva cada pacote neste CSV')
    parser.add_argument('--list', action='store_true', help='lista as portas seriais e sai')
    parser.add_argument('--info', action='store_true', help='lê versão e parâmetros (M0 e M1 no 3V3)')
    parser.add_argument('--padrao', action='store_true', help='grava o padrão de fábrica (M0 e M1 no 3V3)')
    args = parser.parse_args()
    if args.list:
        list_all()
        return 0
    port = choose_port(args.port)
    if args.info or args.padrao:
        return read_module(port, args.padrao)
    return receive(port, args.baud, args.log)


if __name__ == '__main__':
    sys.exit(main())
