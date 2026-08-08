# Complete Room 315 integrated-campaign archive

The Git tree retains the campaign protocol, summary, manifest, source identity,
human-readable logs and per-case verification records. The complete original
campaign directory, including the twelve MCAP recordings, is published as a
GitHub release asset.

- Release tag: `room315-evidence-v2`
- Release page: <https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz/releases/tag/room315-evidence-v2>
- Asset: `room315_integrated_campaign_v2_full_2026-08-07.tar.zst`
- Direct download: <https://github.com/aip-primeca-occitanie/mfja_3rd_floor_gz/releases/download/room315-evidence-v2/room315_integrated_campaign_v2_full_2026-08-07.tar.zst>
- Size: `77,799,300` bytes
- SHA-256: `7775885867f8fde6c23d24e881d339447d5fc622be0143f2be5ca21d7f259b1b`
- Archive root: `room315_integrated_campaign_v2/`
- Contents: 942 regular files and 207 directories

The retained `SHA256SUMS.full-archive` is byte-identical to the checksum list
inside the archive. The repository's `SHA256SUMS.lightweight` independently
verifies its 929 retained payload files and has SHA-256
`6556c061b0ca64be35b52cd2322f77855c4ba365bfac2a80cfe508cca389962a`.
After downloading, verify and extract the bundle with:

```bash
echo '7775885867f8fde6c23d24e881d339447d5fc622be0143f2be5ca21d7f259b1b  room315_integrated_campaign_v2_full_2026-08-07.tar.zst' | sha256sum -c -
tar --zstd -xf room315_integrated_campaign_v2_full_2026-08-07.tar.zst
cd room315_integrated_campaign_v2
sha256sum -c SHA256SUMS
```

To rerun the preserved B03 frame-extraction script without changing its
recorded source, extract the archive under `report/evidence/` so that the
original campaign-relative path is restored.
