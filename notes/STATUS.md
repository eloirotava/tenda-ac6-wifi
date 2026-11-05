# Status — driver aberto Tenda AC6

Último marco aceito: 2026-11-02 — APs 2,4 + 5 GHz WPA2 estáveis no boot.

## O que já está resolvido

1. **PMU 53573 (cold boot 5G)** — recursos 18 e 20 no `max_res_mask` (`0x140000`),
   senão ucode/PLL falham após boot frio.
2. **Bring-up 5G rev56 / 20693** — init + replay (`ac56_wlreplay`); canal de
   trabalho do AP: 149 (com `iw reg set BR`).
3. **Power-save no AP** — `b43_ac_pmq_drain` passa a chamar
   `ieee80211_sta_ps_transition` (sem isso o celular “associa sem internet”).
4. **U-APSD** — desligado no hostapd 5G; cliente cai em PS-poll clássico.
5. **Reset sob upload** — `acphy_ampdu=0x6d2` (RX BA off). Não era fonte:
   TX (download) ok; RX agregado ~15 Mbit/s derrubava.
6. **Cripto HW rev56** — inutilizável para dados → `nohwcrypt=1`.
7. **2,4 GHz** — AP ok em NOHT (HT20 falhava com “extension channel disabled”).

## Dívida técnica (foco deste repo)

- [ ] Substituir `phy_ac_wlreplay56.h` / `phy_ac_wlchan56.h` por init aberto.
- [ ] Achar a causa real do crash com RX A-MPDU e religar agregação.
- [ ] HT no 43217 (2,4).
- [ ] Cripto HW no rev56 (ou documentar por que fica em software).
- [ ] TX HT/VHT no 5G além do legado/HT20 atual.
- [ ] Renomear o módulo / identidade de build para AC6 (hoje ainda `b43.ko`).

## O que este repo deliberadamente não contém

- Patches e árvore do Netgear R7000 / série 880.
- Imagem OpenWrt, DTS de placa, `wl` de fábrica.
- Credenciais, helpers de bancada (TTL/breaker), dumps brutos de rastro.
- Snapshot companion do artigo (`reativa-lab/tenda-ac6-open-wifi`).
