package xiangshan.backend.execute.fu.jmp

import chisel3._
object JumpOpType {
  def jal: UInt = "b00".U
  def jalr: UInt = "b01".U
  def auipc: UInt = "b10".U
  def prefetch_i: UInt = "b11".U
  def dasicscall_j: UInt = "b100".U   // this would split into a dasicscall_j & a csrw
  def dasicscall_jr: UInt = "b101".U  // this would split into a jalr & a csrw
  //    def call = "b11_011".U
  //    def ret  = "b11_100".U
  def jumpOpIsJal(op: UInt): Bool = op === jal
  def jumpOpIsJalr(op: UInt): Bool = op === jalr
  def jumpOpIsAuipc(op: UInt): Bool = op === auipc
  def jumpOpIsPrefetch_I(op: UInt): Bool = op === prefetch_i
  def jumpOpIsDasicscall(op: UInt): Bool = (op === dasicscall_j || op === dasicscall_jr)
  def jumpOpIsDasicscallJR(op: UInt): Bool = op === dasicscall_jr
  def jumpOpIsDasicscallJ(op: UInt): Bool = op === dasicscall_j
}