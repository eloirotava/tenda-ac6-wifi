# Tenda AC6 — driver Wi‑Fi aberto

Desenvolvimento do driver aberto dual-band para o **Tenda AC6 v1** (BCM53573,
64 MB RAM, 8 MB flash): rádio 2,4 GHz (BCM43217 PCIe) + 5 GHz (d11 rev 56 /
radio 20693 no SoC).

Este repositório é a casa do driver do AC6. Não é o projeto genérico `b43`,
nem o trabalho do Netgear R7000, nem a imagem OpenWrt completa.

Imagem / plataforma: [`eloirotava/openwrt-tenda-ac6`](https://github.com/eloirotava/openwrt-tenda-ac6)  
Companion do artigo (snapshot curado, separado): `reativa-lab/tenda-ac6-open-wifi`

## Estado atual (uso aceito)

- Dois APs WPA2 simultâneos (2,4 + 5 GHz), estáveis por dias.
- Vazão aproximada observada: ~4 Mbit/s (2,4) e ~20 Mbit/s (5 GHz).
- Custo conhecido da estabilidade: RX A-MPDU desligado, HT20, software crypto.

### Params de carga

Ver `configuration/ac6-wifi.init`. O essencial:

| Param | Papel |
|---|---|
| `ac56_wlreplay=99741` | bring-up 5G via replay gerado (provisório) |
| `acphy_ampdu=0x6d2` | RX BA off — sem isso upload ~15 Mbit/s reseta o aparelho |
| `nohwcrypt=1` | cripto HW do rev56 não entrega dados |
| hostapd 5G com WMM/U-APSD off | U-APSD do rev56 não entrega; cliente usa PS-poll |

## Layout

```
driver/           fontes do módulo (drop-in em backports …/broadcom/b43)
configuration/    init de carga + AP 5 GHz
patches/          deltas só do AC6 (950–960) sobre a base AC-PHY
tools/            mkreplay / mkchan / rastros de bring-up
deps/cordic.c     cordic fora de árvore (dep de link no build local)
notes/            estado e próximos passos
```

O nome do `.ko` no aparelho ainda é `b43.ko` (compatibilidade com o stack
mac80211/backports). O produto e o foco de desenvolvimento deste repo são o
**AC6**, não a manutenção genérica do b43.

## Dependências geradas (a eliminar)

`driver/phy_ac_wlreplay56.h` e `driver/phy_ac_wlchan56.h` foram gerados a partir
de rastros on-device (`tools/mkreplay.py`, `tools/mkchan.py`). Servem ao
bring-up atual do 5 GHz. **Objetivo aberto:** substituir por init de PHY/rádio
próprio e remover esses headers.

Firmware microcode do rádio e blobs do `wl` de fábrica **não** entram aqui.

## Build (local, aparelho em release 85)

1. SDK/kernel do OpenWrt `tenda-ac6` (linux 6.12.74) + árvore backports casada.
2. Colocar `driver/` em `…/drivers/net/wireless/broadcom/b43`.
3. Compilar `deps/cordic.c` como módulo e o backports (compat → cfg80211 →
   mac80211 → driver) contra o mesmo kernel.
4. Instalar os `.ko` (ou `.ko.gz`) em `/root/k85` no AC6 e habilitar
   `configuration/ac6-wifi.init` + `ac6-5gap.init`.

Detalhes do ambiente de lab e da bancada ficam fora deste repo.

## Próximo trabalho

Ver `notes/STATUS.md`.
